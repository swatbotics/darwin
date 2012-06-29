#include "Joystick.h"
#include <stdexcept>

#include <SDL.h>

int Joystick::_sdlinit = 0;
int Joystick::_numJoysticks = 0;

Joystick::Joystick() {
  _joystick = 0;
  checkInitSDL();
}

Joystick::Joystick(int index) {
  _joystick = 0;
  checkInitSDL();
  open(index);
}

Joystick::Joystick(const std::string& name) {
  _joystick = 0;
  checkInitSDL();
  open(name);
}

Joystick::~Joystick() {
  if (_joystick) { close(); }
  checkQuitSDL();
}

int Joystick::numJoysticks() const {
  return _numJoysticks;
}

std::string Joystick::joystickName(int i) const {

  if (i < 0 || i >= _numJoysticks) {
    throw std::runtime_error("joystick index out of bounds");
  }

  return SDL_JoystickName(i);

}

std::vector<std::string> Joystick::joystickNames() const {
	
  std::vector<std::string> rval;
  for (int i=0; i<_numJoysticks; ++i) {
    rval.push_back(SDL_JoystickName(i));
  }
  return rval;
  
}

void Joystick::open(int index) {
  
  if (index < 0 || index >= _numJoysticks) {
    throw std::runtime_error("joystick index out of bounds");
  }

  if (_joystick) { close(); }
  if (SDL_JoystickOpened(index)) {
    throw std::runtime_error("joystick already open");
  }
  
  _joystick = SDL_JoystickOpen(index);
  _joyIndex = index;

  _numAxes = SDL_JoystickNumAxes(_joystick);
  _numBalls = SDL_JoystickNumBalls(_joystick);
  _numHats = SDL_JoystickNumHats(_joystick);
  _numButtons = SDL_JoystickNumButtons(_joystick);
    
}

void Joystick::open(const std::string& name) {
  for (int i=0; i<_numJoysticks; ++i) {
    if (SDL_JoystickName(i) == name) {
      open(i);
      return;
    }
  }
  throw std::runtime_error("joystick " + name + " not found");
}

void Joystick::close() {
  if (!_joystick) { return; }
  SDL_JoystickClose(_joystick);
  _joystick = 0;
}


bool Joystick::isOpen() const { return _joystick; }

int Joystick::numAxes() const { return _numAxes; }
int Joystick::numBalls() const { return _numBalls; }
int Joystick::numHats() const { return  _numHats; }
int Joystick::numButtons() const { return _numButtons; }

int Joystick::getAxis(int axis) const {
  if (axis < 0 || axis >= _numAxes) {
    throw std::runtime_error("axis index out of bounds");
  }
  return SDL_JoystickGetAxis(_joystick, axis);
}

unsigned char Joystick::getHat(int hat) const {

  if (hat < 0 || hat >= _numHats) {
    throw std::runtime_error("hat index out of bounds");
  }

  unsigned char c = SDL_JoystickGetHat(_joystick, hat);
  unsigned char rval = 0;

  if (c & SDL_HAT_UP)    { rval += JOYSTICK_HAT_UP; }
  if (c & SDL_HAT_RIGHT) { rval += JOYSTICK_HAT_RIGHT; }
  if (c & SDL_HAT_DOWN)  { rval += JOYSTICK_HAT_DOWN; }
  if (c & SDL_HAT_LEFT)  { rval += JOYSTICK_HAT_LEFT; }

  return rval;

}

bool Joystick::getButton(int button) const {
  if (button < 0 || button >= _numButtons) {
    throw std::runtime_error("button index out of bounds");
  }
  return SDL_JoystickGetButton(_joystick, button);
}


void Joystick::getBall(int ball, int& dx, int& dy) const {
  if (ball < 0 || ball >= _numBalls) {
    throw std::runtime_error("ball index out of bounds");
  }
  SDL_JoystickGetBall(_joystick, ball, &dx, &dy);
}

void Joystick::checkInitSDL() {
  if (!_sdlinit) {
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_NOPARACHUTE)) {
      printf("%s\n",SDL_GetError());
      throw std::runtime_error("Error initting SDL in Joystick wrapper");
    }
  }
  _numJoysticks = SDL_NumJoysticks();
  ++_sdlinit;
}

void Joystick::checkQuitSDL() {
  if (--_sdlinit == 0) {
    SDL_Quit();
  }
}

void Joystick::update() const {
  SDL_JoystickUpdate();
}
