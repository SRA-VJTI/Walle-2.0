/**
	BALANCING-BOT-MOTION (BBM) library for WALL-E 2.0
	This module is used for motion of the balancing bot.
	Using this module the bot can be made to move:
		1)	Forward/ Backward
		2)	Left/ Right (by Differential Drive)

	For more information visit http://sra.vjti.info/

	modified 5 June, 2017
	by Society Of Robotics And Automation, VJTI.
*/

/*	Directions	*/
enum dir {
	FORWARD		= 0,
	BACKWARD	= 1,
	RIGHT		= 2,
	LEFT		= 3
};

/*	This Function makes the bot move in the desired direction, which is passed as the first argument,
	with a desired speed which is passed as the second argument 	*/
void move(dir, short, struct initial_data*);

/*	This Function makes the bot move in the desired direction, which is passed as the first argument,
	with a desired speed which is passed as the second argument 	*/
void move(dir, short, float*);

void move_start(struct initial_data*, struct initial_data*);

void move_end(struct initial_data*, struct initial_data*);

void swap_init_data(struct initial_data*, struct initial_data*);