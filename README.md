## Объединение проекта на esp32+radsens mini board+oled+charger с проектом proton bridge.

![case](https://github.com/user-attachments/assets/093fe116-0859-4c3a-9b80-04a46e546e1d)

ProtonBridge arduino скетч для esp32, который позволяет использовать приложение "Дозиметр Atom" на android/ios.

Железный состав проекта:

 1. Трубка СТС-6 (можно любую, но в коде поменять чувствительность)
 2. Плата заряда аккумулятора с I2C-управлением на микросхеме IP5306
 3. OLED дисплей 128x64 0.96 дюймов, I2C, монохромный синий/желтый
    ssd1306
 4. Макетная плата NodeMCU ESP32 (ESP-WROOM-32) CP2102
 5. RadSens Board Mini

Корпус расчитан на СТС-6
![board_in](https://github.com/user-attachments/assets/2fdcf0fc-a005-41fc-95a3-ebd167a5add4)

Ссыка на оригинал ProtonBridge http://forum.rhbz.org/topic.php?forum=80&topic=111&p=3

Ссылка на плату radsens https://climateguard.ru/radsens-board-mini/

TG канал, на котором можно скачать приложение для смартфона https://web.telegram.org/k/#@software_kbradar
![atomswift](https://github.com/user-attachments/assets/e80ecd65-d5c1-4536-89ba-19a945a1121f)
