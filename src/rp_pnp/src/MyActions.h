#ifndef _MY_ACTIONS_
#define _MY_ACTIONS_

// robotname as external variable (defined in MyActions.cpp)
extern std::string robotname;

// Action implementation

void ainit(string params, bool *run);
void gotopose(string params, bool *run);
void home(string params, bool *run);
void wave(string params, bool *run);

void waitstart(string params, bool *run);
void syscheck(string params, bool *run);
void waiterror(string params, bool *run);
void controlcar(string params, bool *run);
void controlreap(string params, bool *run);
void stopcar(string params, bool *run);
void waitclean(string params, bool *run);

int closeToHomeCond(string params);

#endif
