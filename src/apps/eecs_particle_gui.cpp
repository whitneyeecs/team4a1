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
//#include "a1/StateEstimator.hpp"
#include <fstream>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include "mapping/occupancy_grid.hpp"
#include "lcmtypes/maebot_particle_map_t.hpp"

#include "a1/ParticleFilter.hpp"
#include "a1/Explore.hpp"
#include "a1/RobotConstants.hpp"

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
#include "imagesource/image_u8x3.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

class StateHandler {
public:
	// drawing stuff
	eecs467::OccupancyGrid grid;
	eecs467::OccupancyGrid configSpaceGrid;
	image_u8_t* im;
	image_u8_t* configSpaceIm;
	std::vector<float> path;
	std::vector<float> prob_path;
	std::vector<float> pose_path;
	std::vector<float> wayPoints;
	float counter;
	std::ofstream posefile;
	std::ofstream probfile;
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

		counter = 0;

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
		configSpaceIm = nullptr;

		lcm.subscribe("MAEBOT_POSE", &StateHandler::handleLcmMessagePose, this);

		lcm.subscribe("MAEBOT_PARTICLE_MAP", &StateHandler::handleLcmMessage, this);

	}

	~StateHandler() {
		if (im != nullptr) {
			image_u8_destroy(im);
		}

		if (configSpaceIm != nullptr) {
			image_u8_destroy(configSpaceIm);
		}
	}
	
	void handleLcmMessagePose(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_pose_t* msg) {
		
		pthread_mutex_lock(&renderMutex);		

		if(pose_path.size() == 0)
			posefile.open ("a1_pose_position.csv", std::ios::out);

		pose_path.push_back(msg->x);
		pose_path.push_back(msg->y);
		pose_path.push_back(0.0f);

		posefile << msg->x << " " << msg->y << " " << (msg->utime >> 9) << "\n"; 
	
		counter++;
	
		pthread_mutex_unlock(&renderMutex);
	}

	void handleLcmMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const maebot_particle_map_t* msg) {
		
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

		if (configSpaceIm == nullptr) {
			configSpaceIm = image_u8_create(grid.widthInCells(), grid.heightInCells());
		}

		configSpaceGrid.fromLCM(msg->config_space);
		for (unsigned int y = 0; y < configSpaceGrid.heightInCells(); ++y){
			for (unsigned int x = 0; x < configSpaceGrid.widthInCells(); ++x) {
				if (state(configSpaceGrid(x, y)) == eecs467::WALL) {
					// configSpaceIm->buf[y * im->stride + x] = (0x0F << 24) | (0xFF);
					configSpaceIm->buf[y * im->stride + x] = 0;
				} else if (state(configSpaceGrid(x, y)) == eecs467::UNKNOWN) {
					// configSpaceIm->buf[y * im->stride + x] = (0x0F << 24) | (0xFF << 8);
					configSpaceIm->buf[y * im->stride + x] = 126;
				} else {
					configSpaceIm->buf[y * im->stride + x] = 255;
				}
			}
		}

		wayPoints.clear();
		for (int i = 0; i < msg->num_path; ++i) {
			wayPoints.push_back(msg->path_x[i]);
			wayPoints.push_back(msg->path_y[i]);
			wayPoints.push_back(0.02);
		}

		if(prob_path.size() == 0)
			probfile.open ("a1_prob_position.csv", std::ios::out);		

		prob_path.push_back(msg->particles[0].pose.x);
		prob_path.push_back(msg->particles[0].pose.y);
		prob_path.push_back(0.0f);

		probfile << msg->particles[0].pose.x << " " 
			<< msg->particles[0].pose.y << " " 
			<< (msg->particles[0].pose.utime >> 9) << "\n";
	
		path.clear();
		unsigned int i = 0;
		if (path.size() == 0 && msg->num_particles != 0) {
			path.push_back(msg->particles[i].pose.x);
			path.push_back(msg->particles[i].pose.y);
			path.push_back(0.0f);
			i++;
		}

		for ( ; i < msg->particles.size(); ++i) {
			path.push_back(msg->particles[i].pose.x);
			path.push_back(msg->particles[i].pose.y);
			path.push_back(0.0f);
			
			path.push_back(msg->particles[i].pose.x);
			path.push_back(msg->particles[i].pose.y);
			path.push_back(0.0f);

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
				// resize image fromabout:startpage cells to meters
				// then center it
				eecs467::Point<float> origin = state->grid.originInGlobalFrame();
				vx_object_t* vim = vxo_chain(
					vxo_mat_translate3(origin.x, origin.y, 0),
					vxo_mat_scale((double)state->grid.metersPerCell()),
					vxo_image_from_u8(state->im, 0,
					0));
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"), vim);
			}

			if (state->configSpaceIm != nullptr) {
				// resize image fromabout:startpage cells to meters
				// then center it
				eecs467::Point<float> origin = state->grid.originInGlobalFrame();
				vx_object_t* vim = vxo_chain(
					vxo_mat_translate3(origin.x, origin.y, 0),
					vxo_mat_scale((double)state->grid.metersPerCell()),
					vxo_image_from_u8(state->configSpaceIm, 0,
					0));
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"), vim);
			}

			//waypoints
			if (state->wayPoints.size() != 0) {
				int vec_size = state->wayPoints.size();
				vx_resc_t* verts = vx_resc_copyf((state->wayPoints).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_points(verts, vec_size / 3,  vxo_points_style(vx_orange, 2.0f)));
			}

			//particles
			if (state->path.size() != 0) {
				int vec_size = state->path.size();
				vx_resc_t* verts = vx_resc_copyf((state->path).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_points(verts, vec_size / 3,  vxo_points_style(vx_blue, 2.0f)));
			}

			if (state->prob_path.size() != 0) {
				int vec_size = state->prob_path.size();
				vx_resc_t* verts = vx_resc_copyf((state->prob_path).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_points(verts, vec_size / 3,  vxo_points_style(vx_red, 2.0f)));
			}

			if (state->pose_path.size() != 0) {
				int vec_size = state->pose_path.size();
				vx_resc_t* verts = vx_resc_copyf((state->pose_path).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "state"),
					vxo_points(verts, vec_size / 3,  vxo_points_style(vx_black, 2.0f)));
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

