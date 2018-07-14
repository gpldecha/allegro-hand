#ifndef _ROCKSCISSORSPAPER_H
#define _ROCKSCISSORSPAPER_H

#include <bhand/BHand.h>


class RockScissorsPaper{

public:

    static void MotionRock(BHand* pBHand, double *q_des);
    static void MotionScissors(BHand* pBHand, double *q_des);
    static void MotionPaper(BHand* pBHand, double *q_des);

private:

    static void SetGainsRSP(BHand* pBHand);

};

#endif
