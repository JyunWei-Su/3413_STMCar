/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/main_screen/MainViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#include "main.h"

#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
extern UART_HandleTypeDef huart1;
extern int speed_L_T, speed_R_T;
unsigned char uTx;

MainViewBase::MainViewBase() :
    buttonCallback(this, &MainViewBase::buttonCallbackHandler)
{

    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    __background.setPosition(0, 0, 800, 480);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    animatedImage.setXY(0, 0);
    animatedImage.setBitmaps(BITMAP_BKG_01_ID, BITMAP_BKG_18_ID);
    animatedImage.setUpdateTicksInterval(2);
    animatedImage.startAnimation(false, true, true);

    circle.setPosition(240, 80, 320, 320);
    circle.setCenter(160, 160);
    circle.setRadius(160);
    circle.setLineWidth(0);
    circle.setArc(0, 360);
    circlePainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    circle.setPainter(circlePainter);
    circle.setAlpha(180);

    buttonWithLabel_0.setXY(370, 210);
    buttonWithLabel_0.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_0.setLabelText(touchgfx::TypedText(T___SINGLEUSE_R65O));
    buttonWithLabel_0.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_0.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_0.setAction(buttonCallback);

    buttonWithLabel_1.setXY(370, 150);
    buttonWithLabel_1.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_1.setLabelText(touchgfx::TypedText(T___SINGLEUSE_8X8L));
    buttonWithLabel_1.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_1.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_1.setAction(buttonCallback);

    buttonWithLabel_2.setXY(370, 90);
    buttonWithLabel_2.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_2.setLabelText(touchgfx::TypedText(T___SINGLEUSE_YXJU));
    buttonWithLabel_2.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_2.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_2.setAction(buttonCallback);

    buttonWithLabel_3.setXY(370, 270);
    buttonWithLabel_3.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_3.setLabelText(touchgfx::TypedText(T___SINGLEUSE_4V6O));
    buttonWithLabel_3.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_3.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_3.setAction(buttonCallback);

    buttonWithLabel_4.setXY(370, 330);
    buttonWithLabel_4.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_4.setLabelText(touchgfx::TypedText(T___SINGLEUSE_Z300));
    buttonWithLabel_4.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_4.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_4.setAction(buttonCallback);

    buttonWithLabel_5.setXY(430, 210);
    buttonWithLabel_5.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_5.setLabelText(touchgfx::TypedText(T___SINGLEUSE_DPQN));
    buttonWithLabel_5.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_5.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_5.setAction(buttonCallback);

    buttonWithLabel_6.setXY(490, 210);
    buttonWithLabel_6.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_6.setLabelText(touchgfx::TypedText(T___SINGLEUSE_1MFD));
    buttonWithLabel_6.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_6.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_6.setAction(buttonCallback);

    buttonWithLabel_7.setXY(430, 150);
    buttonWithLabel_7.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_7.setLabelText(touchgfx::TypedText(T___SINGLEUSE_TGF0));
    buttonWithLabel_7.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_7.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_7.setAction(buttonCallback);

    buttonWithLabel_8.setXY(430, 270);
    buttonWithLabel_8.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_8.setLabelText(touchgfx::TypedText(T___SINGLEUSE_NXZ9));
    buttonWithLabel_8.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_8.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_8.setAction(buttonCallback);

    buttonWithLabel_9.setXY(310, 210);
    buttonWithLabel_9.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_9.setLabelText(touchgfx::TypedText(T___SINGLEUSE_S410));
    buttonWithLabel_9.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_9.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_9.setAction(buttonCallback);

    buttonWithLabel_a.setXY(250, 210);
    buttonWithLabel_a.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_a.setLabelText(touchgfx::TypedText(T___SINGLEUSE_28UN));
    buttonWithLabel_a.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_a.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_a.setAction(buttonCallback);

    buttonWithLabel_b.setXY(310, 150);
    buttonWithLabel_b.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_b.setLabelText(touchgfx::TypedText(T___SINGLEUSE_YQK9));
    buttonWithLabel_b.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_b.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_b.setAction(buttonCallback);

    buttonWithLabel_c.setXY(310, 270);
    buttonWithLabel_c.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_ICON_BUTTON_PRESSED_ID));
    buttonWithLabel_c.setLabelText(touchgfx::TypedText(T___SINGLEUSE_10FH));
    buttonWithLabel_c.setLabelColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_c.setLabelColorPressed(touchgfx::Color::getColorFromRGB(255, 255, 255));
    buttonWithLabel_c.setAction(buttonCallback);

    textProgress_L_T.setXY(60, 17);
    textProgress_L_T.setProgressIndicatorPosition(0, 0, 180, 37);
    textProgress_L_T.setRange(0, 4, 4, 0);
    textProgress_L_T.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textProgress_L_T.setNumberOfDecimals(0);
    textProgress_L_T.setTypedText(touchgfx::TypedText(T_PROGRESS_TEST));
    textProgress_L_T.setBackground(touchgfx::Bitmap(BITMAP_IMAGE_PROGRESS_BACKGROUND_ID));
    textProgress_L_T.setValue(2);

    textProgress_L_R.setXY(60, 60);
    textProgress_L_R.setProgressIndicatorPosition(0, 0, 180, 37);
    textProgress_L_R.setRange(0, 4, 4, 0);
    textProgress_L_R.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textProgress_L_R.setNumberOfDecimals(0);
    textProgress_L_R.setTypedText(touchgfx::TypedText(T_PROGRESS_TEST));
    textProgress_L_R.setBackground(touchgfx::Bitmap(BITMAP_IMAGE_PROGRESS_BACKGROUND_ID));
    textProgress_L_R.setValue(2);

    textAreaL.setXY(11, 17);
    textAreaL.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaL.setLinespacing(0);
    textAreaL.setTypedText(touchgfx::TypedText(T___SINGLEUSE_0VLC));

    textProgress_R_T.setXY(560, 20);
    textProgress_R_T.setProgressIndicatorPosition(0, 0, 180, 37);
    textProgress_R_T.setRange(0, 4, 4, 0);
    textProgress_R_T.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textProgress_R_T.setNumberOfDecimals(0);
    textProgress_R_T.setTypedText(touchgfx::TypedText(T_PROGRESS_TEST));
    textProgress_R_T.setBackground(touchgfx::Bitmap(BITMAP_IMAGE_PROGRESS_BACKGROUND_ID));
    textProgress_R_T.setValue(2);

    textProgress_R_R.setXY(560, 60);
    textProgress_R_R.setProgressIndicatorPosition(0, 0, 180, 37);
    textProgress_R_R.setRange(0, 4, 4, 0);
    textProgress_R_R.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textProgress_R_R.setNumberOfDecimals(0);
    textProgress_R_R.setTypedText(touchgfx::TypedText(T_PROGRESS_TEST));
    textProgress_R_R.setBackground(touchgfx::Bitmap(BITMAP_IMAGE_PROGRESS_BACKGROUND_ID));
    textProgress_R_R.setValue(2);

    textAreaR.setXY(748, 17);
    textAreaR.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaR.setLinespacing(0);
    textAreaR.setTypedText(touchgfx::TypedText(T___SINGLEUSE_QRWW));

    lineProgress_G_Z.setXY(60, 430);
    lineProgress_G_Z.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_G_Z.setRange(0, 32767);
    lineProgress_G_Z.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_G_ZPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_G_Z.setPainter(lineProgress_G_ZPainter);
    lineProgress_G_Z.setStart(8, 8);
    lineProgress_G_Z.setEnd(172, 8);
    lineProgress_G_Z.setLineWidth(8);
    lineProgress_G_Z.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_G_Z.setValue(0);

    lineProgress_G_Y.setXY(60, 390);
    lineProgress_G_Y.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_G_Y.setRange(0, 32767);
    lineProgress_G_Y.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_G_YPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_G_Y.setPainter(lineProgress_G_YPainter);
    lineProgress_G_Y.setStart(8, 8);
    lineProgress_G_Y.setEnd(172, 8);
    lineProgress_G_Y.setLineWidth(8);
    lineProgress_G_Y.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_G_Y.setValue(0);

    lineProgress_G_X.setXY(60, 352);
    lineProgress_G_X.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_G_X.setRange(0, 32767);
    lineProgress_G_X.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_G_XPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_G_X.setPainter(lineProgress_G_XPainter);
    lineProgress_G_X.setStart(8, 8);
    lineProgress_G_X.setEnd(172, 8);
    lineProgress_G_X.setLineWidth(8);
    lineProgress_G_X.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_G_X.setValue(0);

    lineProgress_A_Z.setXY(560, 429);
    lineProgress_A_Z.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_A_Z.setRange(0, 32767);
    lineProgress_A_Z.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_A_ZPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_A_Z.setPainter(lineProgress_A_ZPainter);
    lineProgress_A_Z.setStart(8, 8);
    lineProgress_A_Z.setEnd(172, 8);
    lineProgress_A_Z.setLineWidth(8);
    lineProgress_A_Z.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_A_Z.setValue(0);

    lineProgress_A_Y.setXY(560, 389);
    lineProgress_A_Y.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_A_Y.setRange(0, 32767);
    lineProgress_A_Y.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_A_YPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_A_Y.setPainter(lineProgress_A_YPainter);
    lineProgress_A_Y.setStart(8, 8);
    lineProgress_A_Y.setEnd(172, 8);
    lineProgress_A_Y.setLineWidth(8);
    lineProgress_A_Y.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_A_Y.setValue(0);

    lineProgress_A_X.setXY(560, 352);
    lineProgress_A_X.setProgressIndicatorPosition(0, 0, 180, 16);
    lineProgress_A_X.setRange(0, 32767);
    lineProgress_A_X.setBackground(touchgfx::Bitmap(BITMAP_LINE_PROGRESS_BACKGROUND_ID));
    lineProgress_A_XPainter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    lineProgress_A_X.setPainter(lineProgress_A_XPainter);
    lineProgress_A_X.setStart(8, 8);
    lineProgress_A_X.setEnd(172, 8);
    lineProgress_A_X.setLineWidth(8);
    lineProgress_A_X.setLineEndingStyle(touchgfx::Line::ROUND_CAP_ENDING);
    lineProgress_A_X.setValue(0);

    textArea1.setXY(306, 0);
    textArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textArea1.setLinespacing(0);
    textArea1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_CA7F));

    add(__background);
    add(animatedImage);
    add(circle);
    add(buttonWithLabel_0);
    add(buttonWithLabel_1);
    add(buttonWithLabel_2);
    add(buttonWithLabel_3);
    add(buttonWithLabel_4);
    add(buttonWithLabel_5);
    add(buttonWithLabel_6);
    add(buttonWithLabel_7);
    add(buttonWithLabel_8);
    add(buttonWithLabel_9);
    add(buttonWithLabel_a);
    add(buttonWithLabel_b);
    add(buttonWithLabel_c);
    add(textProgress_L_T);
    add(textProgress_L_R);
    add(textAreaL);
    add(textProgress_R_T);
    add(textProgress_R_R);
    add(textAreaR);
    add(lineProgress_G_Z);
    add(lineProgress_G_Y);
    add(lineProgress_G_X);
    add(lineProgress_A_Z);
    add(lineProgress_A_Y);
    add(lineProgress_A_X);
    add(textArea1);
}

void MainViewBase::setupScreen()
{

}

void MainViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &buttonWithLabel_0)
    {
        //Interaction0
        //When buttonWithLabel_0 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '0';
        //HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 2;
    	speed_R_T = 2;
        uTx = '0';
        HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_1)
    {
        //Interaction1
        //When buttonWithLabel_1 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '1';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 3;
    	speed_R_T = 3;
    	uTx = '1';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_2)
    {
        //Interaction2
        //When buttonWithLabel_2 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '2';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 4;
    	speed_R_T = 4;
    	uTx = '2';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_3)
    {
        //Interaction3
        //When buttonWithLabel_3 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '3';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 1;
    	speed_R_T = 1;
    	uTx = '3';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_4)
    {
        //Interaction4
        //When buttonWithLabel_4 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '4';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 0;
    	speed_R_T = 0;
    	uTx = '4';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_5)
    {
        //Interaction5
        //When buttonWithLabel_5 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '5';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 3;
    	speed_R_T = 2;
    	uTx = '5';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_6)
    {
        //Interaction6
        //When buttonWithLabel_6 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '6';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 4;
    	speed_R_T = 2;
    	uTx = '6';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_7)
    {
        //Interaction7
        //When buttonWithLabel_7 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '7';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 4;
    	speed_R_T = 3;
    	uTx = '7';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_8)
    {
        //Interaction8
        //When buttonWithLabel_8 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '8';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 0;
    	speed_R_T = 1;
    	uTx = '8';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_9)
    {
        //Interaction9
        //When buttonWithLabel_9 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '9';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 2;
    	speed_R_T = 3;
    	uTx = '9';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_a)
    {
        //InteractionA
        //When buttonWithLabel_a clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = 'a';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 2;
    	speed_R_T = 4;
    	uTx = 'a';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_b)
    {
        //InteractionB
        //When buttonWithLabel_b clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = 'b';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 3;
    	speed_R_T = 4;
    	uTx = 'b';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
    else if (&src == &buttonWithLabel_c)
    {
        //InteractionC
        //When buttonWithLabel_c clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = 'c';
    	//HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 1;
    	speed_R_T = 0;
    	uTx = 'c';
    	HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
}
