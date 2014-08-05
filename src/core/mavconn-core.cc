/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

(c) 2009 MAVCONN PROJECT

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief LCM example
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

// BOOST includes

#include <cstdio>
#include <errno.h>
#include <unistd.h>
#include <glib.h>
#include <stdio.h>
#include <glibtop.h>
#include <glibtop/cpu.h>
#include <glibtop/mem.h>
#include <glibtop/swap.h>
#include <glibtop/proclist.h>
#include <glibtop/netload.h>

// MAVLINK message format includes
#include "mavconn.h"
#include "core/MAVConnParamClient.h"

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// For uptime
#include <sys/sysinfo.h>
// For remaining diskspace
#include <sys/statvfs.h>

// Timer for benchmarking
struct timeval tv;

using std::string;
using namespace std;

#define COMM_READY_TIMEOUT 200
#define COMM_GCS_TIMEOUT 2000

typedef struct
{
	lcm_t* lcm;
	MAVConnParamClient* client;
} thread_context_t;

enum COMM_STATE
{
	COMM_STATE_UNINIT=0,
	COMM_STATE_BOOT,
	COMM_STATE_READY,
	COMM_STATE_GCS_ESTABLISHED
};

enum COMPONENT_STATE
{
	CMP_STATE_UNINIT=0,
	CMP_STATE_OK
};

typedef struct
{
	uint8_t core;
	uint8_t comm;
	uint8_t comm_errors;
	uint8_t subsys_errors;
} system_state_t;

system_state_t static inline mk_system_state_t()
{
	system_state_t s;
	s.core = MAV_STATE_UNINIT;
	s.comm = COMM_STATE_UNINIT;
	s.comm_errors = 0;
	s.subsys_errors = 0;
	return s;
}


// Settings
gint systemid = getSystemID();		///< The unique system id of this MAV, 0-255. Has to be consistent across the system
gint compid = MAV_COMP_ID_SYSTEM_CONTROL;		///< The unique component id of this process.
gint systemType = MAV_TYPE_QUADROTOR;		///< The system type
gboolean silent = false;				///< Wether console output should be enabled
gboolean verbose = false;				///< Enable verbose output
gboolean emitHeartbeat = false;			///< Generate a heartbeat with this process
gboolean emitHealth = false;				///< Emit CPU load as debug message 101
gboolean debug = false;					///< Enable debug functions and output
gboolean cpu_performance = false;		///< Set CPU to performance mode (needs root)
gboolean simulate_vision_with_gps = false;	///< Simulates vision with gps data (distorted and delayed)

mavlink_heartbeat_t heartbeat;

/**** SIMULATOR ****/
mavlink_vision_position_estimate_t pos_vis_simulated;
uint64_t last_sim = 0;
float asctec_last_roll = -1000.f;
float asctec_last_pitch;
float asctec_last_yaw;
/**** SIMULATOR ****/

uint64_t currTime;
uint64_t lastTime;

uint64_t lastGCSTime;

MAVConnParamClient* paramClient;

static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,const mavconn_mavlink_msg_container_t* container, void * user)
{
	if (debug) printf("Received message on channel \"%s\":\n", channel);

	thread_context_t* context = static_cast<thread_context_t*>(user);

	lcm_t* lcm = context->lcm;
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Handle param messages
	context->client->handleMAVLinkPacket(msg);

	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
		if (cmd.target_system == getSystemID())
		{
			switch (cmd.command)
			{
			case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
			{
                if (cmd.param2 == 2)
				{
					if (verbose) printf("Shutdown received, shutting down system\n");
                    if (cmd.confirmation)
                    {
                        mavlink_message_t response;
                        mavlink_command_ack_t ack;
                        ack.command = cmd.command;
						ack.result = 0;
                        mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
                        sendMAVLinkMessage(lcm, &response);
                    }
				}
				else if (cmd.param2 == 1)
				{
					if (verbose) printf("Reboot received, rebooting system\n");
                    if (cmd.confirmation)
                    {
                        mavlink_message_t response;
                        mavlink_command_ack_t ack;
                        ack.command = cmd.command;
						ack.result = 0;
                        mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
                        sendMAVLinkMessage(lcm, &response);
                    }
				}
                usleep(100000);

				int ret = 0;
                if (cmd.param2 == 2)
				{
					ret = system ("shutdown -P 0");
					if (ret) if (verbose) std::cerr << "Shutdown failed." << std::endl;
				}
				else if (cmd.param2 == 1)
				{
					ret = system ("reboot");
					if(ret) if (verbose) std::cerr << "Reboot failed." << std::endl;
				}

				if (cmd.confirmation)
				{
					mavlink_message_t response;
					mavlink_command_ack_t ack;
					ack.command = cmd.command;
					if (ret == 0)
					{
						ack.result = 0;
					}
					else
					{
						ack.result = 1;
					}
					mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
					sendMAVLinkMessage(lcm, &response);
				}
			}
			break;
			}
		}
	}
	break;
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		switch(mavlink_msg_heartbeat_get_type(msg))
		{
		case MAV_TYPE_GCS:
			gettimeofday(&tv, NULL);
			uint64_t currTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
			// Groundstation present
			lastGCSTime = currTime;
			if (verbose) std::cout << "Heartbeat received from GCS/OCU " << msg->sysid;
			break;
		}
	}
	break;
	case MAVLINK_MSG_ID_PING:
	{
		mavlink_ping_t ping;
		mavlink_msg_ping_decode(msg, &ping);
		uint64_t r_timestamp = getSystemTimeUsecs();
		if (ping.target_system == 0 && ping.target_component == 0)
		{
			mavlink_message_t r_msg;
			mavlink_msg_ping_pack(systemid, compid, &r_msg, ping.seq, msg->sysid, msg->compid, r_timestamp);
			sendMAVLinkMessage(lcm, &r_msg);
		}
	}
	break;
	case MAVLINK_MSG_ID_ATTITUDE:
	{
		if (simulate_vision_with_gps)
		{
			if(msg->sysid == getSystemID() && msg->compid == 199)	//only listen to asctec bridge
			{
				mavlink_attitude_t att;
				mavlink_msg_attitude_decode(msg, &att);
				asctec_last_roll = att.roll;
				asctec_last_pitch = att.pitch;
				asctec_last_yaw = att.yaw;
			}
		}
	}
	break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
	{
		if (simulate_vision_with_gps)
		{
			if(msg->sysid == getSystemID() && msg->compid == 199 && asctec_last_roll != -1000.f)	//only listen to asctec bridge
			{
				mavlink_local_position_ned_t pos;
				mavlink_msg_local_position_ned_decode(msg, &pos);

				gettimeofday(&tv, NULL);
				currTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

				if (currTime - last_sim > 1000000)
				{
					if (last_sim > 0)
					{
						mavlink_message_t msg;
						mavlink_msg_vision_position_estimate_encode(getSystemID(), 130, &msg, &pos_vis_simulated);
						sendMAVLinkMessage(lcm, &msg);
					}
					pos_vis_simulated.usec = pos.time_boot_ms;	// this is meant for asctec testing, the asctec bridge sets timestamp to local time in us, so this is ok
					pos_vis_simulated.roll = asctec_last_roll;
					pos_vis_simulated.pitch = asctec_last_pitch;
					pos_vis_simulated.yaw = asctec_last_yaw;
					pos_vis_simulated.x = pos.x;
					pos_vis_simulated.y = pos.y;
					pos_vis_simulated.z = pos.z;
					last_sim = currTime;
				}
			}
		}
	}
	break;
	default:
		if (debug) printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
}

void* lcm_wait(void* lcm_ptr)
				{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle (lcm);
	}
	return NULL;
				}

// Return values:
// -1 for Not available (unknown)
// 0 for error
// 1 for Read only
// 2 for read and write
int check_filesystem()
{
	// XXX path is hardcoded for now
	char *path = "/home/fang/mavconn-testfile.txt";
	int rval;

	/* Check file existence. */
	rval = access (path, F_OK);
	if (rval == 0)
	{
		if (debug) printf ("%s exists\n", path);
	}
	else
	{
		if (errno == ENOENT)
		{
			if (debug) printf ("%s does not exist\n", path);
			return 0;
		}
		else if (errno == EACCES)
		{
			if (debug) printf ("%s is not accessible\n", path);
			return 0;
		}
	}

	/* Check read access. */
	rval = access (path, R_OK);
	if (rval == 0)
	{
		if (debug) printf ("%s is readable\n", path);
	}
	else
	{
		if (debug) printf ("%s is not readable (access denied)\n", path);
		return 0;
	}

	/* Check write access. */
	rval = access (path, W_OK);
	if (rval == 0)
	{
		if (debug) printf ("%s is writable\n", path);
		return 2;
	}
	else if (errno == EACCES)
	{
		if (debug) printf ("%s is not writable (access denied)\n", path);
		return 0;
	}
	else if (errno == EROFS)
	{
		if (debug) printf ("%s is not writable (read-only filesystem)\n", path);
		return 1;
	}
	return 0;
}


int main (int argc, char ** argv)
{
	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "heartbeat", 0, 0, G_OPTION_ARG_NONE, &emitHeartbeat, "Emit Heartbeat", NULL },
			{ "cpu", 'u', 0, G_OPTION_ARG_NONE, &cpu_performance, "Set CPU to performance mode", NULL },
			{ "health", 0, 0, G_OPTION_ARG_NONE, &emitHealth, "Emit system health", NULL },
			{ "silent", 's', 0, G_OPTION_ARG_NONE, &silent, "Be silent", NULL },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", NULL },
			{ "sysid", 'a', 0, G_OPTION_ARG_INT, &systemid, "ID of this system", NULL },
			{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate between LCM broadcast bus and ground control link");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	lcm_t * lcm;

	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	// Initialize parameter client before subscribing (and receiving) MAVLINK messages
	paramClient = new MAVConnParamClient(systemid, compid, lcm, "mavconn-sysctrl.cfg", verbose);
	//paramClient->setParamValue("SYS_ID", systemid);
	//paramClient->readParamsFromFile("px_system_control.cfg");

	thread_context_t thread_context;
	thread_context.lcm = lcm;
	thread_context.client = paramClient;

	mavconn_mavlink_msg_container_t_subscription_t * commSub =
			mavconn_mavlink_msg_container_t_subscribe (lcm, MAVLINK_MAIN, &mavlink_handler, &thread_context);

	// Thread
	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread create failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	// Initialize system information library
	glibtop_init();

	glibtop_cpu cpu;
	glibtop_mem memory;
	glibtop_swap swap;
	glibtop_proclist proclist;
	glibtop_netload netload;

	const char *net_device = "eno1";

	glibtop_get_netload(&netload, net_device);
	uint64_t old_bytes_in = netload.bytes_in;
	uint64_t old_bytes_out = netload.bytes_out;

	glibtop_get_cpu (&cpu);
	uint64_t old_total = cpu.total;
	uint64_t old_idle = cpu.idle;

	uint8_t baseMode = 0;
	uint8_t customMode = 0;
	uint8_t systemStatus = 0;

	printf("\nPX SYSTEM CONTROL STARTED ON MAV %d (COMPONENT ID:%d) - RUNNING..\n\n", systemid, compid);

	while (1)
	{
		gettimeofday(&tv, NULL);
		currTime =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

		if (currTime - lastTime > 1000000)
		{

			if (cpu_performance)
			{
				//set cpu to always full power
				system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
				system("echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor");
				system("echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor");
				system("echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor");
			}

			// SEND OUT TIME MESSAGE
			// send message as close to time aquisition as possible
			mavlink_message_t msg;

			// Send heartbeat if enabled
			if (emitHeartbeat)
			{
				mavlink_msg_system_time_pack(systemid, compid, &msg, currTime, 0);
				sendMAVLinkMessage(lcm, &msg);

				lastTime = currTime;
				if (verbose) std::cout << "Emitting heartbeat" << std::endl;

				// SEND HEARTBEAT

				// Pack message and get size of encoded byte string
				mavlink_msg_heartbeat_pack(systemid, compid, &msg, systemType, MAV_AUTOPILOT_PIXHAWK, baseMode, customMode, systemStatus);
				sendMAVLinkMessage(lcm, &msg);
			}

			if (emitHealth)
			{
				// GET SYSTEM INFORMATION
				glibtop_get_cpu (&cpu);
				glibtop_get_mem(&memory);
				glibtop_get_swap(&swap);
				glibtop_get_netload(&netload, net_device);
				float load_percent = ((float)(cpu.total-old_total)-(float)(cpu.idle-old_idle))
					/ (float)(cpu.total-old_total) * 100.0f;
				old_total = cpu.total;
				old_idle = cpu.idle;

				float memory_percent = ((float)memory.used/(float)memory.total) * 100.0f;

				float swap_percent = ((float)swap.used/(float)swap.total) * 100.0f;

				float bytes_per_s_in = (float)(netload.bytes_in - old_bytes_in)/1024.0f;
				float bytes_per_s_out = (float)(netload.bytes_out - old_bytes_out)/1024.0f;

				old_bytes_in = netload.bytes_in;
				old_bytes_out = netload.bytes_out;

				struct sysinfo info;
				sysinfo(&info);

				int disk_health = check_filesystem();

				struct statvfs fiData;
				// XXX: directory hardcoded for now
				statvfs("/home/fang",&fiData);
				float disk_usage_percent = ((float)(fiData.f_blocks - fiData.f_bfree)/(float)fiData.f_blocks)*100.0f;
				float disk_usage_gb = ((float)(fiData.f_blocks - fiData.f_bfree)*fiData.f_bsize)/1024/1024/1024;
				float disk_total_gb = ((float)fiData.f_blocks*fiData.f_bsize)/1024/1024/1024;




				// get temperature on Odroid XU
				ifstream rawavg("/sys/devices/virtual/thermal/thermal_zone0/temp");
				string temp;
				getline(rawavg, temp);
				float temp_c = (float)(atoi(temp.c_str())/1000);
				rawavg.close();


				if (verbose)
				{
                    cout << "\nBlock size: "<< fiData.f_bsize;
                    cout << "\nTotal no blocks: "<< fiData.f_blocks;
                    cout << "\nFree blocks: "<< fiData.f_bfree;

					printf("CPU TYPE INFORMATIONS \n\n"
							"Cpu Total : %ld \n"
							"Cpu User : %ld \n"
							"Cpu Nice : %ld \n"
							"Cpu Sys : %ld \n"
							"Cpu Idle : %ld \n"
							"Cpu Frequences : %ld \n",
							(unsigned long)cpu.total,
							(unsigned long)cpu.user,
							(unsigned long)cpu.nice,
							(unsigned long)cpu.sys,
							(unsigned long)cpu.idle,
							(unsigned long)cpu.frequency);

					printf("\nLOAD: %f %%\n\n", load_percent);

					printf("\nMEMORY USING\n\n"
							"Memory Total : %ld MiB\n"
							"Memory Used : %ld MiB\n"
							"Memory Free : %ld MiB\n"
							"Memory Shared: %ld MiB\n"
							"Memory Buffered : %ld MiB\n"
							"Memory Cached : %ld MiB\n"
							"Memory user : %ld MiB\n"
							"Memory Locked : %ld MiB\n",
							(unsigned long)memory.total/(1024*1024),
							(unsigned long)memory.used/(1024*1024),
							(unsigned long)memory.free/(1024*1024),
							(unsigned long)memory.shared/(1024*1024),
							(unsigned long)memory.buffer/(1024*1024),
							(unsigned long)memory.cached/(1024*1024),
							(unsigned long)memory.user/(1024*1024),
							(unsigned long)memory.locked/(1024*1024));

					printf("MEMORY: %f %%\n\n", memory_percent);

					printf("\nSWAP\n\n"
							"Swap Total : %ld MiB\n"
							"Swap Used : %ld MiB\n"
							"Swap Free : %ld MiB\n",
							(unsigned long)swap.total/(1024*1024),
							(unsigned long)swap.used/(1024*1024),
							(unsigned long)swap.free/(1024*1024));

					printf("SWAP: %f %%\n\n", swap_percent);

					int which = 0, arg = 0;
					glibtop_get_proclist(&proclist,which,arg);
					// printf("%ld\n%ld\n%ld\n",
					// 		(unsigned long)proclist.number,
					// 		(unsigned long)proclist.total,
					// 		(unsigned long)proclist.size);

					printf("UPTIME: %ld s\n\n", info.uptime);

					printf("Filesystem: %d\n\n", disk_health);

					printf("Disk usage: %f %%\n\n", disk_usage_percent);
					printf("Disk usage: %f GiB\n\n", disk_usage_gb);

					printf("TEMP: %f deg C\n\n", (float)(temp_c));

					printf("NETLOAD: in: %f KiB/s, out: %f KiB/s\n\n", bytes_per_s_in, bytes_per_s_out);
				}

				mavlink_message_t msg;
				// Pack message and get size of encoded byte string
				mavlink_msg_onboard_health_pack(systemid, compid, &msg, (uint32_t)info.uptime,
											(uint16_t)cpu.frequency,
											(uint8_t)load_percent,
											(uint8_t)memory_percent,
											((float)memory.total)/1024.0f/1024.0f/1024.0f,
											(uint8_t)swap_percent,
											((float)swap.total)/1024.0f/1024.0f/1024.0f,
											(int8_t)disk_health,
											(uint8_t)disk_usage_percent,
											disk_total_gb,
											temp_c,
											-1.0f,
											bytes_per_s_in,
											bytes_per_s_out);
				sendMAVLinkMessage(lcm, &msg);

			}
		}

		//sleep as long as possible, if getting near to 1 second sleep only short
		gettimeofday(&tv, NULL);
		uint64_t currTime2 =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		int64_t sleeptime = 900000 - (currTime2-currTime);
		if (sleeptime > 0)
			usleep(sleeptime);
		else
			usleep(10000);
	}

	mavconn_mavlink_msg_container_t_unsubscribe (lcm, commSub);
	lcm_destroy (lcm);

	return 0;
}

