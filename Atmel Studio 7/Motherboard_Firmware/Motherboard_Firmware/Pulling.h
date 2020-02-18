/* 
* Pulling.h
*
* Created: 2/14/2020 4:16:06 PM
* Author: Anthony
*/


#ifndef __PULLING_H__
#define __PULLING_H__


class Pulling
{
//variables
public:
protected:
private:

//functions
public:
	Pulling();
	~Pulling();

	static void SetWheelRPM(float rpm);
	static void IncDecWheelRPM(float rpm);
	static float GetWheelRPM();
protected:
private:
	Pulling( const Pulling &c );
	Pulling& operator=( const Pulling &c );

}; //Pulling

#endif //__PULLING_H__
