#include <ahand_driver/ahandDriver.h>
#include "rock_paper_scissors/RockScissorsPaper.h"

#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>

const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 3;

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

char Getch();
void PrintInstruction();
void MainLoop();

char Getch(){
  char buf=0;
  struct termios old={0};
  fflush(stdout);
  if(tcgetattr(0, &old)<0)
    perror("tcsetattr()");
  old.c_lflag&=~ICANON;
  old.c_lflag&=~ECHO;
  old.c_cc[VMIN]=1;
  old.c_cc[VTIME]=0;
  if(tcsetattr(0, TCSANOW, &old)<0)
    perror("tcsetattr ICANON");
  if(read(0,&buf,1)<0)
    perror("read()");
  old.c_lflag|=ICANON;
  old.c_lflag|=ECHO;
  if(tcsetattr(0, TCSADRAIN, &old)<0)
    perror ("tcsetattr ~ICANON");
  printf("%c\n",buf);
  return buf;
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
  printf("--------------------------------------------------\n");
  printf("myAllegroHand: ");
  if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

  printf("Keyboard Commands:\n");
  printf("H: Home Position (PD control)\n");
  printf("R: Ready Position (used before grasping)\n");
  printf("G: Three-Finger Grasp\n");
  printf("K: Four-Finger Grasp\n");
  printf("P: Two-finger pinch (index-thumb)\n");
  printf("M: Two-finger pinch (middle-thumb)\n");
  printf("E: Envelop Grasp (all fingers)\n");
  printf("A: Gravity Compensation\n\n");

  printf("O: Servos OFF (any grasp cmd turns them back on)\n");
  printf("Q: Quit this program\n");

  printf("--------------------------------------------------\n\n");
}

bool bRun = true;
void MainLoop(BHand* const pBHand, double* q_des)
{
  if(pBHand == NULL)
      return;

  while (bRun)
    {
      int c = Getch();
      switch (c)
        {
        case 'q':
      if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
      bRun = false;
      break;

        case 'h':
      if (pBHand) pBHand->SetMotionType(eMotionType_HOME);
      break;

        case 'r':
      if (pBHand) pBHand->SetMotionType(eMotionType_READY);
      break;

        case 'g':
      if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_3);
      break;

        case 'k':
      if (pBHand) pBHand->SetMotionType(eMotionType_GRASP_4);
      break;

        case 'p':
      if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_IT);
      break;

        case 'm':
      if (pBHand) pBHand->SetMotionType(eMotionType_PINCH_MT);
      break;

        case 'a':
      if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
      break;

        case 'e':
      if (pBHand) pBHand->SetMotionType(eMotionType_ENVELOP);
      break;

        case 'o':
      if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
      break;

    case '1':
      RockScissorsPaper::MotionRock(pBHand, q_des);
      break;

    case '2':
      RockScissorsPaper::MotionScissors(pBHand, q_des);
      break;

    case '3':
      RockScissorsPaper::MotionPaper(pBHand, q_des);
      break;

        }
    }
}

void singal_handler(int s){
    printf("Caught signal %d\n",s);
    bRun = false;
}

int main(int argc, TCHAR* argv[])
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = singal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  PrintInstruction();
  AhandDriver ahandDriver;
  BHand* const pBHand = ahandDriver.getBHand();
  double* q_des = ahandDriver.getDesiredJointPosition();

  MainLoop(pBHand, q_des);


  return 0;
}
