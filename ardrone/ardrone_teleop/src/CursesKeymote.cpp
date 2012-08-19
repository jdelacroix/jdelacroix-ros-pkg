/*
 * CursesKeymote.cpp
 *
 *  Created on: Aug 18, 2012
 *      Author: jpdelacroix
 */

#include "CursesKeymote.h"

CursesKeymote::CursesKeymote() {
	// TODO Auto-generated constructor stub
	m_command_srv_client = m_node_handle.serviceClient<ardrone_driver::ARDroneCommand>("ardrone_send_command");
	m_control_srv_client = m_node_handle.serviceClient<ardrone_driver::ARDroneControl>("ardrone_send_control");

	m_control_effort = 1.0;
}

CursesKeymote::~CursesKeymote() {
	// TODO Auto-generated destructor stub
}

void CursesKeymote::run(void ) {

	ros::Rate loop_rate(30); // 30Hz
	int input_ch;

	while(m_node_handle.ok() && (input_ch = getch()) != 0x20) {
		ros::spinOnce();
		switch (input_ch) {
			case 0x6b: // K - Emergency/Kill
				signalCommand(CMD_EMERGENCY);
				break;
			case 0x74: // T - Takeoff
				signalCommand(CMD_TAKEOFF);
				break;
			case 0x67:
				signalCommand(CMD_LAND);
				break;
			case 0x61: // A - Roll left
				signalControl(CTRL_ROLL, -m_control_effort);
				break;
			case 0x64: // D - Roll right
				signalControl(CTRL_ROLL, m_control_effort);
				break;
			case 0x77: // W - Pitch down (forward)
				signalControl(CTRL_PITCH, -m_control_effort);
				break;
			case 0x73: // S - Pitch up (backward)
				signalControl(CTRL_PITCH, m_control_effort);
				break;
			case 0x71: // Q - Yaw CCW (left)
				signalControl(CTRL_YAW, -m_control_effort);
				break;
			case 0x65: // E - Yaw CW (right)
				signalControl(CTRL_YAW, m_control_effort);
				break;
			case 0x72: // R - Gaz Up
				signalControl(CTRL_GAZ, m_control_effort);
				break;
			case 0x66: // F - Gaz Down
				signalControl(CTRL_GAZ, m_control_effort);
				break;
			case 0x1b: // H - Hover
				signalControl(CTRL_TRIM, 0.0);
				break;
			default:
				//signalControl(CTRL_TRIM, 0.0);
				break;
		}
		loop_rate.sleep();
	}
}

void CursesKeymote::signalControl(ControlSignal sig, float control) {

	m_control_call.request.pitch = 0.0;
	m_control_call.request.roll = 0.0;
	m_control_call.request.yaw = 0.0;
	m_control_call.request.gaz = 0.0;


	switch (sig) {
		case CTRL_ROLL:
			m_control_call.request.roll = control;
			break;
		case CTRL_PITCH:
			m_control_call.request.pitch = control;
			break;
		case CTRL_YAW:
			m_control_call.request.yaw = control;
			break;
		case CTRL_GAZ:
			m_control_call.request.gaz = control;
			break;
		case CTRL_TRIM:
		default:
			break;
	}

	if(m_control_srv_client.call(m_control_call)) {
	  //ROS_INFO("Sent emergency signal!");
	} else {
	  ROS_ERROR("Failed to make service call: 'ardrone_send_command'");
	}
}

void CursesKeymote::signalCommand(CommandSignal sig) {

	m_command_call.request.emergency = false;
	m_command_call.request.takeoff = false;
	m_command_call.request.land = false;

	switch (sig) {
		case CMD_EMERGENCY:
			m_command_call.request.emergency = true;
			break;
		case CMD_TAKEOFF:
			m_command_call.request.takeoff = true;
			break;
		case CMD_LAND:
			m_command_call.request.land = true;
			break;
		case CMD_NOOP:
		default:
			break;
	}

	if(m_command_srv_client.call(m_command_call)) {
	  //ROS_INFO("Sent emergency signal!");
	} else {
	  ROS_ERROR("Failed to make service call: 'ardrone_send_command'");
	}
}

void CursesKeymote::stop(void ) {
	delwin(m_main_window);
	endwin();
	refresh();
}

bool CursesKeymote::initialize(void ) {
	/*  Initialize ncurses  */

	if ((m_main_window = initscr()) == NULL ) {
		ROS_ERROR("Error initializing ncurses.");
		return false;
	}

	noecho();                  		/*  Turn off key echoing                 */
	keypad(m_main_window, TRUE);	/*  Enable the keypad for non-char keys  */
	nodelay(m_main_window, TRUE);

	/*  Print a prompt and refresh() the screen  */

//	mvprintw(1,  0,	".---.");
//	mvprintw(2,  0,	"|ESC|");
//	mvprintw(3,  0,	"|   |");
//	mvprintw(4,  0,	"'---'");
	mvprintw(1,  0,	"        .---. .---. .---. .---. .---,");
	mvprintw(2,  0,	"        |Q  | |W  | |E  | |R  | |T  |");
	mvprintw(3,  0,	"        |   | |   | |   | |   | |   |");
	mvprintw(4,  0,	"        '---' '---' '---' '---' '---'");
	mvprintw(5,  0,	"         .---. .---. .---. .---. .---. .---.       .---.");
	mvprintw(6,  0,	"         |A  | |S  | |D  | |F  | |G  | |H  |       |K  |");
	mvprintw(7,  0,	"         |   | |   | |   | |   | |   | |   |       |   |");
	mvprintw(8,  0,	"         '---' '---' '---' '---' '---' '---'       '---'");
	mvprintw(9,  0,	"                       .------------------------------.");
	mvprintw(10, 0,	"                       |SPACE                         |");
	mvprintw(11, 0,	"                       |                              |");
	mvprintw(12, 0,	"                       '------------------------------'");
	mvprintw(13, 0,	"   Space - Quit");
	mvprintw(14, 0,	" W,A,S,D - Roll/Pitch     Q,E - Yaw      R/F - Up/Down");
	mvprintw(15, 0,	"       T - Takeoff          G - Land       K - Kill");

	refresh();

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "keymote");

	CursesKeymote keymote;

	while(keymote.initialize() != true) {}
	keymote.run();
	keymote.stop();

	return 1;
}
