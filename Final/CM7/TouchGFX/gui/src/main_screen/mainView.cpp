#include <gui/main_screen/MainView.hpp>

extern int speed_L_T, speed_L_R, speed_R_T, speed_R_R;
extern int G_X, G_Y, G_Z, A_X, A_Y, A_Z;

MainView::MainView()
{
}

void MainView::setupScreen()
{
    //tickCounter = 0;

    //boxProgress.getRange(boxProgressMin, boxProgressMax);
    //imageProgress1.getRange(imageProgress1Min, imageProgress1Max);
    //imageProgress2.getRange(imageProgress2Min, imageProgress2Max);
    //circleProgress.getRange(circleProgressMin, circleProgressMax);
    lineProgress_G_X.getRange(lineProgressMin, lineProgressMax);
    lineProgress_G_Y.getRange(lineProgressMin, lineProgressMax);
    lineProgress_G_Z.getRange(lineProgressMin, lineProgressMax);
    lineProgress_A_X.getRange(lineProgressMin, lineProgressMax);
    lineProgress_A_Y.getRange(lineProgressMin, lineProgressMax);
    lineProgress_A_Z.getRange(lineProgressMin, lineProgressMax);
    //tiltedLineProgress.getRange(tiltedLineProgressMin, tiltedLineProgressMax);
    //textProgress_L_T.getRange(textProgressMin, textProgressMax);
    //textProgress_R_T.getRange(textProgressMin, textProgressMax);
}

void MainView::tearDownScreen()
{

}

void MainView::updateProgress(uint16_t tick)
{
    //boxProgress.setValue(tick % (boxProgressMax + 1));
    textProgress_L_T.setValue(speed_L_T);
    textProgress_R_T.setValue(speed_R_T);
    textProgress_L_R.setValue(speed_L_R);
    textProgress_R_R.setValue(speed_R_R);
    //imageProgress1.setValue(tick % (imageProgress1Max + 1));
    //imageProgress2.setValue(tick % (imageProgress2Max + 1));
    //circleProgress.setValue(tick % (circleProgressMax + 1));
    lineProgress_G_X.setValue(G_X);
    lineProgress_G_Y.setValue(G_Y);
    lineProgress_G_Z.setValue(G_Z);
    lineProgress_A_X.setValue(A_X);
    lineProgress_A_Y.setValue(A_Y);
    lineProgress_A_Z.setValue(A_Z);
    //lineProgress_G_Y.setValue(20000);
    //tiltedLineProgress_L_T.setValue(tick % (tiltedLineProgressMax + 1));
}

void MainView::updateDirection(uint16_t tick)
{
	/*
    if (tick % (boxProgressMax + 1) == 0)
    {
        boxProgress.setDirection((AbstractDirectionProgress::DirectionType)((tick / (boxProgressMax + 1)) % 4));
    }

    if (tick % (imageProgress1Max + 1) == 0)
    {
        imageProgress1.setDirection((tick / (imageProgress1Max + 1)) % 2 == 0 ? AbstractDirectionProgress::RIGHT : AbstractDirectionProgress::LEFT);
    }

    if (tick % (imageProgress2Max + 1) == 0)
    {
        imageProgress2.setDirection((tick / (imageProgress2Max + 1)) % 2 == 0 ? AbstractDirectionProgress::RIGHT : AbstractDirectionProgress::LEFT);
    }
    */
    /*
    if (tick % (circleProgressMax + 1) == 0)
    {
        // Toggle number of steps in circle updates
        int unused1;
        uint16_t unused2;
        uint16_t steps;
        //circleProgress.getRange(unused1, unused1, steps, unused2);

        if (steps == 12)
        {
            //circleProgress.setRange(0, 500, 500, 1);
        }
        else
        {
            //circleProgress.setRange(0, 500, 12, 1);
        }
    }*/
}

void MainView::handleTickEvent()
{
    //tickCounter++;
    updateProgress(tickCounter);
    //updateDirection(tickCounter);
}
