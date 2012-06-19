#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

struct _SDL_Joystick;
typedef struct _SDL_Joystick SDL_Joystick;

#include <string>
#include <vector>

class Joystick {
public:

  enum {
    JOYSTICK_HAT_UP    = 0x01,
    JOYSTICK_HAT_DOWN  = 0x02,
    JOYSTICK_HAT_LEFT  = 0x04,
    JOYSTICK_HAT_RIGHT = 0x08,
  };

  Joystick();
  
  Joystick(int index);
  Joystick(const std::string& name);

  ~Joystick();

  int numJoysticks() const;
  std::string joystickName(int i) const;
  std::vector<std::string> joystickNames() const;

  void open(int index);
  void open(const std::string& name);

  void close();

  bool isOpen() const;

  int numAxes() const;
  int numBalls() const;
  int numHats() const;
  int numButtons() const;
  
  void update() const;
  int getAxis(int axis) const; 
  unsigned char getHat(int hat) const;
  bool getButton(int button) const;
  void getBall(int ball, int& dx, int& dy) const;

private:

  static void checkInitSDL();
  static void checkQuitSDL();

  static int _sdlinit;
  static int _numJoysticks;

  SDL_Joystick* _joystick;
  int _joyIndex;

  int _numAxes;
  int _numBalls;
  int _numHats;
  int _numButtons;
  
};

#endif
