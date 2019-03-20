#include <TimerOne.h>
#include <OneWire.h>
#include <iarduino_RTC.h>
#include <LedControl.h>
// НЕ ИСПОЛЬЗУЙТЕ 9 ПИН!
/* Пины экрана */
#define DATA_IN 7
#define CLK 4
#define LOAD_CS 6
//Порядковый номер экрана на шине
#define NUM_DEV 1
//Адрес дисплея
#define DISP_ADDR 0
/* Пины RGB светодиода */
#define RED_PIN 10
#define GREEN_PIN 5
#define BLUE_PIN 11
/* Датчик температуры */
#define TEMP_PIN 12
// Кнопка смены режимов (настройка времени)
#define MODE_PIN 2
#define ADD_PIN 3
/*
 * Текущий режим:
 * 0 - стандартная работа часов
 * 1 - установка часов
 * 2 - установка минут
 * 3 - установка секунд
 */
volatile byte mode = 0;
// volatile int state = LOW;

//Класс часов
iarduino_RTC time(RTC_DS3231);
//Класс дисплея
LedControl Display = LedControl(DATA_IN, CLK, LOAD_CS, NUM_DEV);
//Класс температурного датчика
OneWire ds(TEMP_PIN);

void setup()
{
    //RGB пины на выход
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    //Выводим дисплей из спящего режима
    Display.shutdown(DISP_ADDR, false);
    //Устанавливаем яркость свечения сегментов
    //Доступные значения от 0 до 15
    Display.setIntensity(DISP_ADDR, 15);
    //Очищаем дисплей
    Display.clearDisplay(DISP_ADDR);
    //Подключаем программное прерывание на 500 мс
    Timer1.initialize(500000);
    Timer1.attachInterrupt(timerTickISR);
    //Инициализируем RTC
    time.begin();

    // Кнопки на прерывания
    pinMode(MODE_PIN, INPUT_PULLUP);
    pinMode(ADD_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MODE_PIN), changeMode, LOW); // привязываем 0-е
    attachInterrupt(digitalPinToInterrupt(ADD_PIN), addButton, LOW); // привязываем 1-е
}

void loop()
{
    //0.0 >= hue <= 1.0 - плавная смена цвета
    static float hue = 0.0;
    setColor(hsv2rgb(hue, 1.0, 1.0));
      hue += 0.002;
      if (hue >= 1.0)
          hue = 0.0;
    delay(200);
}

void addButton()
{
    static unsigned long millis_prev = millis();
    if(millis() - 50 > millis_prev)
    {
        if (mode == 1)
            addSecond();
        else if (mode == 2)
            addMinute();
        else if (mode == 3)
            addHour();
    }
    millis_prev = millis();
}
void addSecond()
{
    if (time.seconds < 59)
        time.seconds += 1;
    else
        time.seconds = 0;
}
void addMinute()
{
    if (time.minutes < 59)
        time.minutes += 1;
    else
        time.minutes = 0;
}
void addHour()
{
    if (time.Hours < 23)
        time.Hours += 1;
    else
        time.Hours = 0;
}
void setTime()
{
    time.settime(time.seconds,time.minutes,time.Hours);
}

void changeMode() {
    static unsigned long millis_prev = millis();
    if(millis() - 80 > millis_prev)
    {
        if (mode == 3)
        {
            setTime();
            mode = 0;
        }
        else
            mode += 1;
    }
    millis_prev = millis();
}

void timerTickISR() {
    //Закодированные цифры для вывода на дисплей функцией setRow()
    const byte Digits[10] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B };
    //Выводимая на дисплей строка
    static byte StrMessage[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    //Делитель времени, каждый count == 500мс
    static byte count = 0;
    static byte isHide = 0;
    // Обычный вывод часов/температуры
    if (mode == 0)
    {
        //Запрашиваем текущее время у модуля DS1302
        time.gettime();

        //Каждую секунду, 10 секунд - часы
        if (count <= 20 && count % 2 == 0)
        {
            //Размещаем значение в выводимой строке
            StrMessage[0] = Digits[time.seconds % 10];
            StrMessage[1] = Digits[time.seconds / 10];
            StrMessage[3] = Digits[time.minutes % 10];
            StrMessage[4] = Digits[time.minutes / 10];
            StrMessage[6] = Digits[time.Hours % 10];
            StrMessage[7] = Digits[time.Hours / 10];
        
            if (isHide) {
                StrMessage[2] = 0;
                StrMessage[5] = 0;
            }
            else {
                StrMessage[2] = 1;
                StrMessage[5] = 1;
            }
            //Выводим строку на дисплей и попутно очищаем StrMessage
            for (int digit = 0; digit < 8; digit++)
            {
                Display.setRow(DISP_ADDR, digit, StrMessage[digit]);
                StrMessage[digit] = 0;
            }
            isHide = !isHide;
        }
        //5 секунд - температура
        /* Датчик меряет температуру ~1сек. Для этого в "нулевую" секунду
        *  мы даем команду замерять температуру. А уже с 1 по 5 - выводим
        *  вместе с замером.
        */
        if (count >= 20 && count % 2 == 0)
        {
            //Если не "нулевая" секунда
            if (count != 20)
            {
                //Очистим сегменты с 7 по 5, так как они могут быть пустыми.
                //Остальные точно перезапишутся в коде ниже
                Display.setRow(DISP_ADDR, 5, 0);
                Display.setRow(DISP_ADDR, 6, 0);
                Display.setRow(DISP_ADDR, 7, 0);
                //Выводим температуру
                float temp = getTemperature();
                //Разбиваем градусы на цифры
                StrMessage[2] = ((byte)(temp * 100)) % 10;
                StrMessage[3] = ((byte)(temp * 10)) % 10;
                StrMessage[4] = ((byte)temp) % 10;
                StrMessage[5] = (byte)(temp / 10);

                //Выводим символы градусов Цельсия
                Display.setRow(DISP_ADDR, 0, B01001110);
                Display.setRow(DISP_ADDR, 1, B01100011);

                //Если отрицательная температура - ставим минус
                if (temp < 0.0)
                {
                    //Если температура >-10C и <10C то минус ставим в 5 ячейку, иначе в 6
                    if (StrMessage[5] == 0)
                        Display.setChar(DISP_ADDR, 5, '-', false);
                    else
                        Display.setChar(DISP_ADDR, 6, '-', false);
                }
                //Выводим градусы
                Display.setDigit(DISP_ADDR, 2, StrMessage[2], false);
                Display.setDigit(DISP_ADDR, 3, StrMessage[3], false);
                //Ставим точку
                Display.setDigit(DISP_ADDR, 4, StrMessage[4], true);
                if (StrMessage[5] != 0)
                    Display.setDigit(DISP_ADDR, 5, StrMessage[5], false);

                //Очищаем StrMessage
                for (int digit = 0; digit < 8; digit++)
                    StrMessage[digit] = 0;
            }
            //Даем команду на замер температуры
            ds.reset();
            ds.write(0xCC);
            ds.write(0x44);
        }
    }
    //Установка времени
    else
    {
        if (count % 2 == 0)
        {
            //Размещаем значение в выводимой строке
            StrMessage[0] = Digits[time.seconds % 10];
            StrMessage[1] = Digits[time.seconds / 10];
            StrMessage[3] = Digits[time.minutes % 10];
            StrMessage[4] = Digits[time.minutes / 10];
            StrMessage[6] = Digits[time.Hours % 10];
            StrMessage[7] = Digits[time.Hours / 10];
            StrMessage[2] = 1;
            StrMessage[5] = 1;

            if (isHide)
            {
                if (mode == 1)
                {
                    StrMessage[0] = 0;
                    StrMessage[1] = 0;
                }
                else if (mode == 2)
                {
                    StrMessage[3] = 0;
                    StrMessage[4] = 0;
                }
                else if (mode == 3)
                {
                    StrMessage[6] = 0;
                    StrMessage[7] = 0;
                }
            }
            //Выводим строку на дисплей и попутно очищаем StrMessage
            for (int digit = 0; digit < 8; digit++)
            {
                Display.setRow(DISP_ADDR, digit, StrMessage[digit]);
                StrMessage[digit] = 0;
            }
            isHide = !isHide;
        }
    }
    //Прошли 15сек цикл
    if (count >= 31)
        count = 0;
    else
        count++;
}
float getTemperature()
{
    byte data[2];
    //Даем команду на выдачу замеренной температуры
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    //Читаем
    data[0] = ds.read();
    data[1] = ds.read();
    //Преобразуем в цельсии
    return (float)((data[1] << 8) + data[0]) / 16.0;    
}
//Устанавливаем цвет на RGB светодиоде
void setColor(float* rgb)
{
    analogWrite(RED_PIN, (int)((1.0 - rgb[0]) * 255));
    analogWrite(GREEN_PIN, (int)((1.0 - rgb[1]) * 255));
    analogWrite(BLUE_PIN, (int)((1.0 - rgb[2]) * 255));
}

//HSV->RGB conversion based on GLSL version
//expects hsv channels defined in 0.0 .. 1.0 interval
float fract(float x) {
    return x - int(x);
}
float mix(float a, float b, float t) {
    return a + (b - a) * t;
}
float* hsv2rgb(float h, float s, float b) {
    static float rgb[3];
    rgb[0] = b * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
    rgb[1] = b * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
    rgb[2] = b * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
    return rgb;
}