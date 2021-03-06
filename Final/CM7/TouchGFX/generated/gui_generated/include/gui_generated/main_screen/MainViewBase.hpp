/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef MAINVIEWBASE_HPP
#define MAINVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/main_screen/MainPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/AnimatedImage.hpp>
#include <touchgfx/widgets/canvas/Circle.hpp>
#include <touchgfx/widgets/canvas/PainterRGB888.hpp>
#include <touchgfx/widgets/ButtonWithLabel.hpp>
#include <touchgfx/containers/progress_indicators/TextProgress.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/containers/progress_indicators/LineProgress.hpp>
#include "main.h"

class MainViewBase : public touchgfx::View<MainPresenter>
{
public:
    MainViewBase();
    virtual ~MainViewBase() {}
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::AnimatedImage animatedImage;
    touchgfx::Circle circle;
    touchgfx::PainterRGB888 circlePainter;
    touchgfx::ButtonWithLabel buttonWithLabel_0;
    touchgfx::ButtonWithLabel buttonWithLabel_1;
    touchgfx::ButtonWithLabel buttonWithLabel_2;
    touchgfx::ButtonWithLabel buttonWithLabel_3;
    touchgfx::ButtonWithLabel buttonWithLabel_4;
    touchgfx::ButtonWithLabel buttonWithLabel_5;
    touchgfx::ButtonWithLabel buttonWithLabel_6;
    touchgfx::ButtonWithLabel buttonWithLabel_7;
    touchgfx::ButtonWithLabel buttonWithLabel_8;
    touchgfx::ButtonWithLabel buttonWithLabel_9;
    touchgfx::ButtonWithLabel buttonWithLabel_a;
    touchgfx::ButtonWithLabel buttonWithLabel_b;
    touchgfx::ButtonWithLabel buttonWithLabel_c;
    touchgfx::TextProgress textProgress_L_T;
    touchgfx::TextProgress textProgress_L_R;
    touchgfx::TextArea textAreaL;
    touchgfx::TextProgress textProgress_R_T;
    touchgfx::TextProgress textProgress_R_R;
    touchgfx::TextArea textAreaR;
    touchgfx::LineProgress lineProgress_G_Z;
    touchgfx::PainterRGB888 lineProgress_G_ZPainter;
    touchgfx::LineProgress lineProgress_G_Y;
    touchgfx::PainterRGB888 lineProgress_G_YPainter;
    touchgfx::LineProgress lineProgress_G_X;
    touchgfx::PainterRGB888 lineProgress_G_XPainter;
    touchgfx::LineProgress lineProgress_A_Z;
    touchgfx::PainterRGB888 lineProgress_A_ZPainter;
    touchgfx::LineProgress lineProgress_A_Y;
    touchgfx::PainterRGB888 lineProgress_A_YPainter;
    touchgfx::LineProgress lineProgress_A_X;
    touchgfx::PainterRGB888 lineProgress_A_XPainter;
    touchgfx::TextArea textArea1;

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<MainViewBase, const touchgfx::AbstractButton&> buttonCallback;

    /*
     * Callback Handler Declarations
     */
    void buttonCallbackHandler(const touchgfx::AbstractButton& src);

    /*
     * Canvas Buffer Size
     */
    static const uint16_t CANVAS_BUFFER_SIZE = 12000;
    uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];
};

#endif // MAINVIEWBASE_HPP
