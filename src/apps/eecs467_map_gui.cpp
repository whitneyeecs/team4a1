#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <glib.h>
#include <gtk/gtk.h>

// c++
#include <vector>
#include "a1/StateEstimator.hpp"

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include "mapping/occupancy_grid.hpp"

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

#include "imagesource/image_u32.h"
#include "imagesource/image_u8.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

class StateHandler {
public:
	// drawing stuff
	eecs467::OccupancyGrid grid;
	image_u8_t* im;
	std::vector<float> path;
	std::vector<float> lidar_rays;
	pthread_mutex_t renderMutex;

	// lcm
	lcm::LCM lcm;
	pthread_t lcm_pid;

	// vx stuff
	vx_application_t vxapp;
	zhash_t * vxlayers;
	vx_world_t* vxworld;      // where vx objects are live
	vx_event_handler_t* vxeh; // for getting mouse, key, and touch events
	vx_gtk_display_source_t* appwrap;
	pthread_mutex_t vxmutex;
	pthread_t render_pid;

public:
	StateHandler() {
		vxworld = vx_world_create();
		vxlayers = zhash_create(sizeof(vx_display_t*),
			sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
		vxapp.impl = this;
		vxapp.display_started = display_started;
		vxapp.display_finished = display_finished;

		if (pthread_mutex_init(&vxmutex, NULL)) {
			printf("state mutex not initialized\n");
			exit(1);
		}

		if (pthread_mutex_init(&renderMutex, NULL)) {
			printf("render mutex not initialized\n");
			exit(1);
		}

		if (!lcm.good()) {
			printf("lcm unable to initialize\n");
			exit(1);
		}

		im = nullptr;

		lcm.subscribe("MAEBOT_MAP_DATA", &StateHandler::handleLcmMessage, this);

	}

	~StateHandler() {
		if (im != nullptr) {
			image_u8_destroy(im);
		}
	}

	void handleLcmMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_map_data_t* msg) {
		
		pthread_mutex_lock(&renderMutex);
		grid.fromLCM(msg->grid);
		// assume grid sizes never change
		if (im == nullptr) {
			im = image_u8_create(grid.widthInCells(), grid.heightInCells());
		}

		for (unsigned int y = 0; y < grid.heightInCells(); ++y){
			for (unsigned int x = 0; x < grid.widthInCells(); ++x) {
				im->buf[y * im->stride + x] = (uint8_t) (-grid(x, y) + 127);
			}
		}

		unsigned int i = 0;
		if (path.size() == 0 && msg->path_num != 0) {
			path.push_back(msg->path_x[i]);
			path.push_back(msg->path_y[i]);
			path.push_back(0.0f);
			i++;
		}

		for ( ; i < msg->path_x.size(); ++i) {
			path.push_back(msg->path_x[i]);
			path.push_back(msg->path_y[i]);
			path.push_back(0.0f);

			path.push_back(msg->path_x[i]);
			path.push_back(msg->path_y[i]);
			path.push_back(0.0f);
		}


		lidar_rays.clear();
		for (int i = 0; i < msg->scan.num_ranges; ++i) {
			lidar_rays.push_back(msg->scan.x_pos[i]);
			lidar_rays.push_back(msg->scan.y_pos[i]);
			lidar_rays.push_back(0);

			float x_end = msg->scan.x_pos[i] + 
				msg->scan.ranges[i] * cos(msg->scan.thetas[i]);
			float y_end = msg->scan.y_pos[i] + 
				msg->scan.ranges[i] * sin(msg->scan.thetas[i]);
			lidar_rays.push_back(x_end);
			lidar_rays.push_back(y_end);
			lidar_rays.push_back(0);	
		}

		pthread_mutex_unlock(&renderMutex);
	}

	void launchThreads() {
		// lcm handle thread
		pthread_create(&lcm_pid, NULL, &StateHandler::lcmHandleThread, this);

		// render thread
		pthread_create(&render_pid, NULL, &StateHandler::renderThread, this);
	}

private:
	static void display_finished(vx_application_t * app, vx_display_t * disp) {
		StateHandler * state = (StateHandler *) app->impl;
		pthread_mutex_lock(&state->vxmutex);

		vx_layer_t * layer = NULL;

		// store a reference to the world and layer that we associate with each vx_display_t
		zhash_remove(state->vxlayers, &disp, NULL, &layer);

		vx_layer_destroy(layer);

		pthread_mutex_unlock(&state->vxmutex);
	}

	static void display_started(vx_application_t * app, vx_display_t * disp) {
		StateHandler * state = (StateHandler *) app->impl;

		vx_layer_t* layer = vx_layer_create(state->vxworld);
		vx_layer_set_display(layer, disp);

		pthread_mutex_lock(&state->vxmutex);
		// store a reference to the world and layer that we associate with each vx_display_t
		zhash_put(state->vxlayers, &disp, &layer, NULL, NULL);
		pthread_mutex_unlock(&state->vxmutex);
	}

	static void* lcmHandleThread(void* arg) {
		StateHandler* state = (StateHandler*) arg;

		while(1) {
			state->lcm.handle();
		}
		return NULL;
	}

	static void* renderThread(void* arg) {
		StateHandler* state = (StateHandler*) arg;

		while (1) {
			pthread_mutex_lock(&state->renderMutex);
			
			if (state->im != nullptr) {
				// resize image from cells to meters
				// then center it
				eecs467::Point<float> origin = state->grid.originInGlobalFrame();
				vx_object_t* vim = vxo_chain(
					vxo_mat_translate3(origin.x, origin.y, 0),
					vxo_mat_scale((double)state->grid.metersPerCell()),
					vxo_image_from_u8(state->im, 0,
					0));
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"), vim);
			}

			if (state->path.size() != 0) {
				int vec_size = state->path.size();
				vx_resc_t* verts = vx_resc_copyf((state->path).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_lines(verts, vec_size / 3, GL_LINES, vxo_lines_style(vx_red, 2.0f)));
			}

			if (state->lidar_rays.size() != 0) {
				int vec_size = state->lidar_rays.size();
				vx_resc_t* verts = vx_resc_copyf((state->lidar_rays).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_lines(verts, vec_size / 3, GL_LINES,
						vxo_lines_style(vx_green, 2.0f)));
			}

			pthread_mutex_unlock(&state->renderMutex);
			
			vx_buffer_swap(vx_world_get_buffer(state->vxworld, "state"));

			usleep(10000);
		}
		return NULL;
	}
};

StateHandler state;


int main(int argc, char* argv[]) {
	gdk_threads_init();
	gdk_threads_enter();
	gtk_init(&argc, &argv);
	state.appwrap = vx_gtk_display_source_create(&state.vxapp);
	GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	GtkWidget *canvas = vx_gtk_display_source_get_widget(state.appwrap);
	gtk_window_set_default_size(GTK_WINDOW(window), 400, 400);
	gtk_container_add(GTK_CONTAINER(window), canvas);
	gtk_widget_show(canvas);
	gtk_widget_show(window);

	vx_global_init();

	state.launchThreads();

	gtk_main();
	gdk_threads_leave();

	vx_global_destroy();
}

