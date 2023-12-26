![RoboBoard-ESP32](https://github.com/LaskaKit/RoboBoard-ESP32/blob/main/img/LaskaKit-roboboard-esp32-2.jpg)

# LaskaKit RoboBoard ESP32

Postav robota! Neboj, není to nic složitého. S naší deskou RoboBoard-ESP32 to bude hračka. RoboBoard totiž má na sobě všechno důležité pro jednoduchého robota a časem z něj můžeš udělat i mnohem komplexnějšího robota díky rozšiřujícím konektorům pro I2C i SPI sběrnici. 

Připojit tak můžeš bezpočet dalších periférií. 

Základem celé desky je ESP32 - ten v sobě skrývá velký výkon, možnost využít Wi-Fi nebo Bluetooth rozhraní a také velkou komunitu bastlířů, kteří s ním vymysleli už spoustu projektů. Nejdeš do neznámé vody - ESP32 je jeden z nejznámějších a nejpoužívanějších modulů vůbec. 

RoboBoard-ESP32 má na sobě už připravený programátor (CH9102F), stačí zapojit USB-C kabel do desky a tvého počítače, spustit Arduino IDE a můžeš začít programovat. Převodník-programátor se postará o nahrání programu do ESP32. 

![RoboBoard-ESP32 pinout](https://github.com/LaskaKit/RoboBoard-ESP32/blob/main/img/RoboBoard-ESP32.JPG)

RoboBoard-ESP32 může řídt až 4 DC motory pomocí PWM, díky driveru TB6612. Maximální trvalý proud skrze jeden kanál je 1A. Driver DC motorů TB6612 je řízen přes PCA9685, který je ovládán skrze I2C sběrnici. Takže ti stačí dva dráty (I2C, SDA - GPIO21 a SCL - GPIO22) k tomu, abys řídil až 4 DC motory. 

Pokud nechceš používat DC motory nebo naopak bys k nim chtěl přidat Serva, RoboBoard-ESP32 je připraven i na tuto možnost. 
Až 8 servo motorů můžeš připojit - k čemu servo motory použiješ už je jenom na tobě. Jednotlivé servo motory řídíš GPIO16 až GPIO19, dále pak GPIO25 a GPIO26 a poslední GPIO32 a GPIO33. 

Co dalšího jsme na desce ještě připravili? Osadili jsme bzučák, ten můžeš ovládat skrze GPIO27. Pak jsme osadili dva I2C uŠup konektor pro připojení I2C čidel, které jsou napájeny 3.3V (SDA - GPIO21 a SCL - GPIO22) a jeden SPI uŠup pro připojení dalších čidel či displejů napájených z 3.3V (CS - GPIO15, SDI/MOSI - GPIO13, CLK - GPIO14, SDO/MISO - GPIO12).

Pro baterii máme připraven standardní tří pinový konektor JST-XH-3P, kam zapojíš 2S LiPol baterie - například tuto https://www.laskakit.cz/rc-lipol-baterie-2s-1000mah-7-4v-20c-jst-bec/

Motory jsou napájené přes spínací step-down měnič LMR14050 z 2S baterie, který dokáže dodat až 5A. 

Napětí baterie, přes dělič napětí, je přivedeno a měřeno GPIO34. 

Vzorové kódy budou připraveny na našem github na adrese https://github.com/LaskaKit/RoboBoard-ESP32/tree/main/SW

