/*
 Prototipo de automacao residencial - Arduino  
 Autor: Victor Santos
 Data: 27/07/2015
 */

#include <LiquidCrystal.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// definicao das portas para cada periferico 
#define LED1 11
#define LED2 12
#define LED3 A4
#define LED4 A5
#define MOTOR1 2
#define MOTOR2 3
#define ADC_BTN A0
#define TERMOPAR_PISCINA A1
#define TERMOPAR_TELHADO A2

//botoes no shield do LCD
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//definicoes do termistor
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000    

//variaveis globais
int old_button = 0;
char fsm_aq_state = 0;
char fsm_ft_state = 0;
char fsm_tp_state = 0;
char fsm_ah_state = 0;
char fsm_ini_state = 0;
char last_ini_state = 0;

int ajuste_tp = 0;
int hora_ajustada = 0, min_ajustado = 0, h_inicio = 0, h_fim = 0;
char flag_aq_automatico = 0;
char jump_fsm_filtragem = 0;

//Pinagem do LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//Array simbolo grau
byte grau[8] = { B00001100, B00010010, B00010010, B00001100, B00000000, B00000000, B00000000, B00000000 };

struct relogio {
    int hora_atual;
    int minuto_atual;
    int hora_prg_inicial;
    int hora_prg_final;
};

struct relogio relogio_sfw;

//maquina de estados para a funcionalidade de aquecimento
enum fsm_aq {
    st_fn_aq, st_ch_aq_on, st_aq_on, st_ch_aq_aut, st_aq_aut, st_aq_aut_tp, st_ch_aq_off, st_aq_off, st_ch_filtr
};

//maquina de estados para a funcionalidade de filtragem
enum fsm_ft {
   st_fn_filt, st_ch_filt_on, st_filt_on, st_ch_filt_aut, st_filt_aut, st_ft_aut_relogio, st_ch_filt_off, st_filt_off

};

//maquina de estados para a funcao de ajuste de temperatura limite do termopar
enum fsm_ajuste_tp {
    st_tp_blink = 0, st_tp_up, st_tp_down, st_tp_stop_blink
};

enum states {
    inicial = 0, aquecimento, filtragem
};

//maquina de estados para a funcao de ajuste de relogio e horario de aquecimento
enum fsm_ajuste_hora {
    st_ar_hora_blink = 0,
    st_hora_up,
    st_hora_down,
    st_ar_min_blink,
    st_min_up,
    st_min_down,
    st_hl_inicio_blink,
    st_inicio_up,
    st_inicio_down,
    st_hl_fim_blink,
    st_fim_up,
    st_fim_down,
    st_ar_stop_blink
};

//estados de funcionamento de cada modo
enum estados {
    ligado = 0, automatico, desligado
};

//funcionalidades do software
enum funcionalidades {
    linha_cima = 0, linha_baixo
};

//enum relogio
enum relogio_elements {
    hora = 0, minuto, hora_inicial, hora_final
};

enum estados_iniciais {
    st_inicial = 0, st_aq, st_flt
};

//termopares
enum termopares {
    piscina = 0, telhado
};

//funcao que faz a leitura dos botoes do shield do LCD pelo ADC
int read_ADC_btn() {

    int i, adc_btn, adc_sum;
    int btn_result;

    adc_sum = 0;
    for (i = 0; i < 4; i++) {
        adc_sum += analogRead(ADC_BTN);
    }
    adc_btn = adc_sum / 4;

    if (adc_btn > 1000)
        btn_result = btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
    // For V1.1 us this threshold
    else if (adc_btn < 50)
        btn_result = btnRIGHT;
    else if (adc_btn < 250)
        btn_result = btnUP;
    else if (adc_btn < 450)
        btn_result = btnDOWN;
    else if (adc_btn < 650)
        btn_result = btnLEFT;
    else if (adc_btn < 850)
        btn_result = btnSELECT;
    else
        btn_result = btnNONE;

    return btn_result;  // when all others fail, return this...
}

void set_motor_state(uint8_t motor, uint8_t value) {
    digitalWrite(motor, value);
}

void set_led_state(uint8_t led, uint8_t value) {
    digitalWrite(led, value);
}

//funcao de calculo do termistor NTC 10k
//calcula a temperatura to termistor com base na leitura do pino informado
int calc_termistor_temp(int thermistor_pin) {
    uint8_t i;
    float average;
    int samples[NUMSAMPLES];

    // take N samples in a row, with a slight delay
    for (i = 0; i < NUMSAMPLES; i++) {
        samples[i] = 1024 - analogRead(thermistor_pin);
        delay(10);
    }

    // average all the samples out
    average = 0;
    for (i = 0; i < NUMSAMPLES; i++) {
        average += samples[i];
    }
    average /= NUMSAMPLES;

    //Serial.print("Average analog reading "); 
    //Serial.println(average);

    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    //Serial.print("Thermistor resistance "); 
    //Serial.println(average);

    float steinhart;
    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    //Serial.print("Temperature "); 
    //Serial.print(steinhart);
    //Serial.println(" *C");
    return (int) steinhart;
}

//retorna a temperatura do termopar, especificar a localizacao do termopar
int temp_termopar(int pos_termopar) {

    float temperatura = 0;

    if (pos_termopar == piscina) {
#ifdef USE_LM35  
        int adc_raw = 0;
        adc_raw = analogRead(TERMOPAR_PISCINA);
        temperatura = (adc_raw * 0.00488);
        temperatura = temperatura * 10;
        Serial.print("Temperatura piscina: ");
        Serial.println(temperatura);

#else  
        temperatura = calc_termistor_temp(TERMOPAR_PISCINA);
#endif
    } else if (pos_termopar == telhado) {

#ifdef USE_LM35  
        int adc_raw = 0;
        adc_raw = analogRead(TERMOPAR_TELHADO);
        temperatura = (adc_raw * 0.00488);
        temperatura = temperatura * 10;
        Serial.print("Temperatura telhado: ");
        Serial.println(temperatura);
#else     
        temperatura = calc_termistor_temp(TERMOPAR_TELHADO);
#endif  
    }
    return (int) temperatura;
}

void ajusta_tp() {

    int button, button2, pressed_button;
    char flag_tp_ajustado = 0;

    while (!flag_tp_ajustado) {

        button = read_ADC_btn();
        if (button != old_button) {
            delay(75);        // debounce button
            button2 = read_ADC_btn();

            if (button == button2) {
                old_button = button;
                pressed_button = button;
                Serial.println(pressed_button); //debug
            }
        }

        switch (fsm_tp_state) {

        /* funcionalidades de aumentro e decrescimo */
        case st_tp_blink:
            TP_blink(100, 1);
            if (pressed_button == btnUP) {
                fsm_tp_state = st_tp_up;
            } else if (pressed_button == btnDOWN) {
                fsm_tp_state = st_tp_down;
            } else if (pressed_button == btnRIGHT) {
                fsm_tp_state = st_tp_stop_blink;
            }
            break;

        case st_tp_up:
            //aumenta a variavel e volta a ficar piscando
            if (ajuste_tp < 50)
                ajuste_tp++;
            lcd_atualiza_tp(ajuste_tp);
            fsm_tp_state = st_tp_blink;
            break;

        case st_tp_down:
            //decresce a variavel a volta a ficar piscando
            if (ajuste_tp > 0)
                ajuste_tp--;
            lcd_atualiza_tp(ajuste_tp);
            fsm_tp_state = st_tp_blink;
            break;

        case st_tp_stop_blink:
            TP_blink(0, 0);
            fsm_tp_state = st_tp_blink;
            flag_tp_ajustado = 1;
            break;

        default:
            break;
        }
    }
}

//ajusta relogio e horario limite do atuador
void ajusta_horario() {

    int button, button2, pressed_button;
    char flag_ah_ajustado = 0;

    while (!flag_ah_ajustado) {

        button = read_ADC_btn();
        if (button != old_button) {
            delay(75);        // debounce button
            button2 = read_ADC_btn();

            if (button == button2) {
                old_button = button;
                pressed_button = button;
                Serial.println(pressed_button); //debug
            }
        }

        switch (fsm_ah_state) {

        /* funcionalidades de aumentro e decrescimo */
        case st_ar_hora_blink:
            relogio_blink(100, hora, relogio_sfw.hora_atual, 1);
            if (pressed_button == btnUP) {
                fsm_ah_state = st_hora_up;
            } else if (pressed_button == btnDOWN) {
                fsm_ah_state = st_hora_down;
            } else if (pressed_button == btnLEFT) {
                fsm_ah_state = st_ar_min_blink;
            }
            break;

        case st_hora_up:
            if (relogio_sfw.hora_atual < 23)
                relogio_sfw.hora_atual++;
            lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);
            fsm_ah_state = st_ar_hora_blink;
            break;

        case st_hora_down:
            if (relogio_sfw.hora_atual > 0)
                relogio_sfw.hora_atual--;
            lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);
            fsm_ah_state = st_ar_hora_blink;
            break;

        case st_ar_min_blink:
            relogio_blink(100, minuto, relogio_sfw.minuto_atual, 1);
            lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);

            if (pressed_button == btnUP) {
                fsm_ah_state = st_min_up;
            } else if (pressed_button == btnDOWN) {
                fsm_ah_state = st_min_down;
            } else if (pressed_button == btnLEFT) {
                fsm_ah_state = st_hl_inicio_blink;
            }
            break;

        case st_min_up:
            if (relogio_sfw.minuto_atual < 59)
                relogio_sfw.minuto_atual++;
            lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);
            fsm_ah_state = st_ar_min_blink;
            break;

        case st_min_down:
            if (relogio_sfw.minuto_atual > 0)
                relogio_sfw.minuto_atual--;
            lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);
            fsm_ah_state = st_ar_min_blink;
            break;

        case st_hl_inicio_blink:
            relogio_blink(100, hora_inicial, relogio_sfw.hora_prg_inicial, 1);

            if (pressed_button == btnUP) {
                fsm_ah_state = st_inicio_up;
            } else if (pressed_button == btnDOWN) {
                fsm_ah_state = st_inicio_down;
            } else if (pressed_button == btnLEFT) {
                fsm_ah_state = st_hl_fim_blink;
            }
            break;

        case st_inicio_up:
            if (relogio_sfw.hora_prg_inicial < 23)
                relogio_sfw.hora_prg_inicial++;
            lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);
            fsm_ah_state = st_hl_inicio_blink;
            break;

        case st_inicio_down:
            if (relogio_sfw.hora_prg_inicial > 0)
                relogio_sfw.hora_prg_inicial--;
            lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);
            fsm_ah_state = st_hl_inicio_blink;
            break;

        case st_hl_fim_blink:
            relogio_blink(100, hora_final, relogio_sfw.hora_prg_final, 1);
            lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);

            if (pressed_button == btnUP) {
                fsm_ah_state = st_fim_up;
            } else if (pressed_button == btnDOWN) {
                fsm_ah_state = st_fim_down;
            } else if (pressed_button == btnLEFT) {
                fsm_ah_state = st_ar_stop_blink;
            }
            break;

        case st_fim_up:
            if (relogio_sfw.hora_prg_final < 23)
                relogio_sfw.hora_prg_final++;
            lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);
            fsm_ah_state = st_hl_fim_blink;
            break;

        case st_fim_down:
            if (relogio_sfw.hora_prg_final > 0)
                relogio_sfw.hora_prg_final--;
            lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);
            fsm_ah_state = st_hl_fim_blink;
            break;

        case st_ar_stop_blink:
            relogio_blink(0, 0, 0, 0);
            fsm_ah_state = st_ar_hora_blink;
            flag_ah_ajustado = 1;
            break;

        default:
            break;
        }
    }
}

//padding para imprimir no LCD numeros com uma quantidade fixa de casas
void padPrint(int value, int width) {
    // pads values with leading zeros to make the given width
    char valueStr[6]; // large enough to hold an int
    itoa(value, valueStr, 10);
    int len = strlen(valueStr);
    if (len < width) {
        len = width - len;
        while (len--)
            lcd.print('0');
    }
    lcd.print(valueStr);
}

//parametros, tempo de blink (para estado = 1), blink = 1, no_blink = 0, valor a piscar, parametro a piscar (hora, minuto, hora_inicial, hora_final
void relogio_blink(int time_ms, int posicao, uint8_t valor, uint8_t state) {
    if (state == 1) {
        switch (posicao) {
        case hora:
            lcd.setCursor(0, 0);
            padPrint(valor, 2);
            delay(time_ms);
            lcd.setCursor(0, 0);
            lcd.print("  ");
            delay(time_ms);
            break;

        case minuto:
            lcd.setCursor(3, 0);
            padPrint(valor, 2);
            delay(time_ms);
            lcd.setCursor(3, 0);
            lcd.print("  ");
            delay(time_ms);
            break;

        case hora_inicial:
            lcd.setCursor(6, 0);
            padPrint(valor, 2);
            delay(time_ms);
            lcd.setCursor(6, 0);
            lcd.print("  ");
            delay(time_ms);
            break;

        case hora_final:
            lcd.setCursor(9, 0);
            padPrint(valor, 2);
            delay(time_ms);
            lcd.setCursor(9, 0);
            lcd.print("  ");
            delay(time_ms);
            break;

        default:
            break;
        }

    } else if (state == 0) {
        lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);
        lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);
    }
}

//funcoes de acesso ao lcd
void lcd_set_intervalo_aq(int hora_inicial, int hora_final) {
    lcd.setCursor(6, 0);
    padPrint(hora_inicial, 2);
    lcd.setCursor(9, 0);
    padPrint(hora_final, 2);
}

void lcd_relogio_atual(int hora, int minuto) {
    lcd.setCursor(0, 0);
    padPrint(hora, 2);
    lcd.setCursor(3, 0);
    padPrint(minuto, 2);
}

void lcd_atualiza_temp_piscina(int temperatura) {
    lcd.setCursor(0, 1);
    lcd.print(temperatura);
}

void lcd_atualiza_tp(int tp) {
    lcd.setCursor(8, 1);
    lcd.print(tp);
}

void lcd_escreve_estado(char * estado, uint8_t linha_lcd) {
    if (linha_lcd == linha_baixo) {
        lcd.setCursor(12, 1);
        lcd.print(estado);
    } else if (linha_lcd == linha_cima) {
        lcd.setCursor(12, 0);
        lcd.print(estado);
    }
}

//inicia toda a estrutura do lcd, conforme o desenho enviado
void lcd_init_struct() {
    //relogio
    lcd.setCursor(2, 0);
    lcd.print(":");

    //separador programacao timer
    lcd.setCursor(8, 0);
    lcd.print("-");

    //grau e celsius piscina
    lcd.setCursor(2, 1);
    //Mostra o simbolo do grau formado pelo array
    lcd.write((byte) 0);
    lcd.print("C");

    //separador programacao timer
    lcd.setCursor(6, 1);
    lcd.print("TP");

    //grau e celsius
    lcd.setCursor(10, 1);
    //Mostra o simbolo do grau formado pelo array
    lcd.write((byte) 0);
}

//parametros, tempo de blink (para estado = 1), blink = 1, no_blink = 0
void TP_blink(int time_ms, uint8_t state) {
    if (state == 1) {
        lcd.setCursor(6, 1);
        lcd.print("TP");
        delay(time_ms);
        lcd.setCursor(6, 1);
        lcd.print("  ");
        delay(time_ms);
    } else if (state == 0) {
        lcd.setCursor(6, 1);
        lcd.print("TP");
    }
}

void (*last_fn_aq)();
void (*last_fn_flt)();

//funcoes de aquecimento,
void aq_ligado() {
    set_led_state(LED1, HIGH);
    set_led_state(LED2, LOW);
    set_motor_state(MOTOR1, HIGH);
}

void aq_automatico() {
    set_led_state(LED1, LOW);
    set_led_state(LED2, HIGH);

    if ((temp_termopar(piscina) < temp_termopar(telhado)) && (temp_termopar(piscina) <= ajuste_tp)) {
        Serial.println("automatico_high");
        set_motor_state(MOTOR1, HIGH);
    } else {
        Serial.println("automatico_low");
        set_motor_state(MOTOR1, LOW);
    }
}

void aq_desligado() {
    set_led_state(LED1, LOW);
    set_led_state(LED2, LOW);
    set_motor_state(MOTOR1, LOW);
}

//funcoes de filtragem
void flt_ligado() {
    set_led_state(LED3, HIGH);
    set_led_state(LED4, LOW);
    set_motor_state(MOTOR2, HIGH);
}

void flt_automatico() {
    set_led_state(LED3, LOW);
    set_led_state(LED4, HIGH);

    if ((relogio_sfw.hora_prg_final > relogio_sfw.hora_atual) && (relogio_sfw.hora_atual >= relogio_sfw.hora_prg_inicial)) {
        set_motor_state(MOTOR2, HIGH);
    } else {
        set_motor_state(MOTOR2, LOW);
    }
}

void flt_desligado() {
    set_led_state(LED3, LOW);
    set_led_state(LED4, LOW);
    set_motor_state(MOTOR2, LOW);
}

void setup() {

    /* iniciando as portas */
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    pinMode(ADC_BTN, INPUT);
    pinMode(TERMOPAR_PISCINA, INPUT);
    pinMode(TERMOPAR_TELHADO, INPUT);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    digitalWrite(MOTOR1, LOW);
    digitalWrite(MOTOR2, LOW);

    /* iniciando o diplay */
    lcd.begin(16, 2);              // start the library
    lcd.clear();
    //Cria o caractere customizado com o simbolo do grau
    lcd.createChar(0, grau);
    lcd.setCursor(4, 0);
    lcd.print("Arduino");
    lcd.setCursor(4, 1);
    lcd.print("SmartSP");
    delay(2000);
    lcd.clear();
    lcd_init_struct();

    Serial.begin(9600);  //turn on serial communication
    Serial.print("st_inicial");

    lcd_atualiza_temp_piscina(temp_termopar(piscina));
    ajuste_tp = 30;
    lcd_atualiza_tp(ajuste_tp);

    relogio_sfw.hora_atual = 0;
    relogio_sfw.minuto_atual = 0;
    lcd_relogio_atual(relogio_sfw.hora_atual, relogio_sfw.minuto_atual);

    relogio_sfw.hora_prg_inicial = 0;
    relogio_sfw.hora_prg_final = 0;
    lcd_set_intervalo_aq(relogio_sfw.hora_prg_inicial, relogio_sfw.hora_prg_final);

    last_fn_aq = aq_desligado;
    (*last_fn_aq)();
    last_fn_flt = flt_desligado;

    lcd_escreve_estado("eINI", linha_baixo);
    lcd_escreve_estado("ESC:", linha_cima);

    fsm_aq_state = st_fn_aq;
    fsm_ft_state = st_fn_filt;
    fsm_tp_state = st_tp_blink;
    fsm_ah_state = st_ar_hora_blink;
    fsm_ini_state = st_inicial;
    last_ini_state = inicial;
}

void loop() {

    int button, button2, pressed_button;

    button = read_ADC_btn();
    if (button != old_button) {
        delay(50);        // debounce button
        button2 = read_ADC_btn();

        if (button == button2) {
            old_button = button;
            pressed_button = button;
            //Serial.println(pressed_button); //debug
        }
    }

    if (last_ini_state == inicial) {
        switch (fsm_ini_state) {
        case st_inicial:
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("eINI", linha_baixo);
            Serial.println("st_inicial");
            if (pressed_button == btnSELECT) {
                fsm_ini_state = st_aq;
            }
            break;

        case st_aq:
            Serial.println("st_aq");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("eAQC", linha_baixo);
            if (pressed_button == btnLEFT) {
                fsm_aq_state = st_fn_aq;
                last_ini_state = aquecimento;
            } else if (pressed_button == btnSELECT) {
                fsm_ini_state = st_flt;
            }
            break;

        case st_flt:
            Serial.println("st_flt");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("eFLT", linha_baixo);
            if (pressed_button == btnLEFT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = filtragem;
            } else if (pressed_button == btnSELECT) {
                fsm_ini_state = st_inicial;
            }
            break;

        default:
            break;

        }
    }

    if (last_ini_state == aquecimento) {

        //maquina de estados aquecimento
        switch (fsm_aq_state) {

        case st_fn_aq:
            Serial.println("st_fn_aq");
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("fAQC", linha_baixo);

            if (pressed_button == btnSELECT) {
                fsm_aq_state = st_ch_aq_on;
            }
            break;

        case st_ch_aq_on:
            Serial.println("st_ch_aq_on");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("aLIG", linha_baixo);

            if (pressed_button == btnSELECT) {
                fsm_aq_state = st_ch_aq_aut;
            } else if (pressed_button == btnLEFT) {
                fsm_aq_state = st_aq_on;
            } else if (pressed_button == btnRIGHT) {
                fsm_aq_state = st_fn_aq;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_aq_on:
            Serial.println("st_aq_on");
            last_fn_aq = aq_ligado;
            fsm_aq_state = st_fn_aq;
            last_ini_state = inicial;
            fsm_ini_state = st_inicial;
            break;

        case st_ch_aq_aut:
            Serial.println("st_ch_aq_aut");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("aAUT", linha_baixo);

            if (pressed_button == btnSELECT) {
                fsm_aq_state = st_ch_aq_off;
            } else if (pressed_button == btnLEFT) {
                fsm_aq_state = st_aq_aut;
            } else if (pressed_button == btnRIGHT) {
                fsm_aq_state = st_fn_aq;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_aq_aut:
            Serial.println("st_aq_aut");
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("aAUT", linha_baixo);

            last_fn_aq = aq_automatico;

            if (pressed_button == btnLEFT) {
                fsm_aq_state = st_aq_aut_tp;
            }

            else if (pressed_button == btnSELECT) {
                fsm_aq_state = st_ch_aq_off;
            }

            else if (pressed_button == btnRIGHT) {
                fsm_aq_state = st_fn_aq;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_aq_aut_tp:
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("a_TP", linha_baixo);
            Serial.println("st_ch_aut_tp");
            ajusta_tp();
            fsm_aq_state = st_aq_aut;
            break;

        case st_ch_aq_off:
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("aDES", linha_baixo);
            Serial.println("st_ch_aq_off");
            if (pressed_button == btnSELECT) {
                fsm_aq_state = st_ch_aq_on;
            } else if (pressed_button == btnLEFT) {
                fsm_aq_state = st_aq_off;
            } else if (pressed_button == btnRIGHT) {
                fsm_aq_state = st_fn_aq;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            } 
            break;

        case st_aq_off:
            Serial.println("st_aq_off");
            last_fn_aq = aq_desligado;
            fsm_aq_state = st_fn_aq;
            last_ini_state = inicial;
            fsm_ini_state = st_inicial;
            break;

        default:
            break;
        }
    }

    if (last_ini_state == filtragem) {

        //maquina de estado filtragem
        switch (fsm_ft_state) {

        case st_fn_filt:
            Serial.println("st_fn_filt");
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("fFLT", linha_baixo);
            if (pressed_button == btnSELECT) {
                fsm_ft_state = st_ch_filt_on;
            } else if (pressed_button == btnRIGHT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_ch_filt_on:
            Serial.println("st_ch_filt_on");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("fLIG", linha_baixo);
            if (pressed_button == btnSELECT) {
                fsm_ft_state = st_ch_filt_aut;
            } else if (pressed_button == btnLEFT) {
                fsm_ft_state = st_filt_on;
            } else if (pressed_button == btnRIGHT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_filt_on:
            Serial.println("st_filt_on");
            last_fn_flt = flt_ligado;
            fsm_ft_state = st_fn_filt;
            last_ini_state = inicial;
            fsm_ini_state = st_inicial;
            break;

        case st_ch_filt_aut:
            Serial.println("st_ch_filt_aut");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("fAUT", linha_baixo);

            if (pressed_button == btnSELECT) {
                fsm_ft_state = st_ch_filt_off;
            } else if (pressed_button == btnLEFT) {
                fsm_ft_state = st_filt_aut;
            } else if (pressed_button == btnRIGHT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_filt_aut:
            Serial.println("st_filt_aut");
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("fAUT", linha_baixo);

            if (pressed_button == btnLEFT) {
                fsm_ft_state = st_ft_aut_relogio;
            }

            if (pressed_button == btnSELECT) {
                fsm_ft_state = st_ch_filt_off;
            }
            
            if (pressed_button == btnRIGHT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            
            last_fn_flt = flt_automatico;
            
            break;

        case st_ft_aut_relogio:
            Serial.println("st_ft_aut_relogio");
            lcd_escreve_estado("FUN:", linha_cima);
            lcd_escreve_estado("fAJR", linha_baixo);
            ajusta_horario();
            fsm_ft_state = st_filt_aut;
            break;

        case st_ch_filt_off:
            Serial.println("st_ch_filt_off");
            lcd_escreve_estado("ESC:", linha_cima);
            lcd_escreve_estado("fDES", linha_baixo);
            if (pressed_button == btnSELECT) {
                fsm_ft_state = st_ch_filt_on; 
            } else if (pressed_button == btnLEFT) {
                fsm_ft_state = st_filt_off;
            } else if (pressed_button == btnRIGHT) {
                fsm_ft_state = st_fn_filt;
                last_ini_state = inicial;
                fsm_ini_state = st_inicial;
            }
            break;

        case st_filt_off:
            Serial.println("st_filt_off");
            last_fn_flt = flt_desligado;
            fsm_ft_state = st_fn_filt;
            last_ini_state = inicial;
            fsm_ini_state = st_inicial;
            break;

        default:
            break;
        }
    }

    lcd_atualiza_temp_piscina(temp_termopar(piscina));
    (*last_fn_aq)();
    (*last_fn_flt)();

    delay(100);
}


