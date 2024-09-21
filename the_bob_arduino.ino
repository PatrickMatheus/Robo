
// Bibliotecas

#include <WiFi.h>
#include <AsyncTCP.h> // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <ArduinoJson.h> // https://arduinojson.org/

#include <RoboCore_Vespa.h>
#include <NewPing.h>
// --------------------------------------------------
// Variaveis

// web server assincrono na porta 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED
const uint8_t PINO_LED = 15;
const uint8_t PINO_HCSR04_ECHO = 26;
const uint8_t PINO_HCSR04_TRIGGER = 25;
bool global_ativo = false;

// JSON aliases
const char *ALIAS_ANGULO = "angulo";
const char *ALIAS_ACTIVE = "active";

const char *ALIAS_DISTANCIA = "distancia";
const char *ALIAS_VELOCIDADE = "velocidade";
const char *ALIAS_VBAT = "vbat";

// variaveis da Vespa
VespaMotors motores;
VespaBattery vbat;
const uint32_t TEMPO_ATUALIZACAO_VBAT = 5000; // [ms]
uint32_t timeout_vbat;
const uint32_t TEMPO_ATUALIZACAO_DISTANCIA = 100; // [ms]
uint32_t timeout_distancia;
uint32_t distancia;
bool active;
// colisao 

//declaracao dos pinos conectados aos sensores de linha
const int SENSOR_LINHA_ESQUERDO = 36;
const int SENSOR_LINHA_DIREITO = 39;

//declaracao das variaveis que armazenam as leituras dos sensores
int leitura_esquerdo = 0;
int leitura_direito = 0;

//declaracao da variavel que armazena o valor de corte para as leituras dos sensores
const int LEITURA_LINHA = 3000;
//declaracao da variavel que armazena a velocidade em linha reta do robo
const int VELOCIDADE = 80;
//declaracao da variavel que armazena o valor que sera somado a velocidade de rotacao dos motores
const int VELOCIDADE_SOMA = 20;
//declaracao da variavel que armazena o valor que sera subtraido da valocidade de rotacao dos motores
const int VELOCIDADE_SUBTRACAO = 50;
//declaracao da variavel que armazena o valor maximo de contagem de parada
const int CONTAGEM_MAXIMA = 100;
//declaracao da variavel do contador para parar o robo caso ele fuja da pista
int contador_parada = 0;

//declaracao dos pinos do sensor
const int PINO_TRIGGER = 25;
const int PINO_ECHO = 26;
//declaracao da variavel que armazena a distancia do obstaculo
const int DISTANCIA_OBSTACULO = 7;
//declaracao da variavel que armazena a distancia maxima de leitura do sensor (300 cm = 3 m)
const int DISTANCIA_MAXIMA = 300;
//declaracao da variavel que armazena o tempo de espera entre leituras do sensor
const int ESPERA = 30;
//declaracao da variavel que armazena o tempo que os movimentos de desvio irao durar
const int ESPERA_MOVIMENTO = 400;
//declaracao da variavel que armazena a distancia medida pelo sensor
int distancia_2;

//declaracao do objeto "sensor_ultrassonico" para a leitura do sensor
NewPing sensor_ultrassonico(PINO_TRIGGER, PINO_ECHO, DISTANCIA_MAXIMA);

// --------------------------------------------------
// Pagina web principal

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>
        TheBob Joystick
    </title>

    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, minimum-scale=1.0, maximum-scale=1.0, user-scalable=0">

    <style>
        html, body {width: 100%; height: 100%; padding: 0; margin: 0;}
        
        body {
            background-color: black;
            padding: 12px;
            border: 0;
            box-sizing: border-box;
            font-family: 'Inter', sans-serif;
            overflow: hidden;
        }

        .container {
            height: 26px;
            width: 50px;
            position: relative;
        }
        .container * {
            position: absolute;
        }

        .battery {
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            height: 20px;
            width: 40px;
            border: 2px solid #F1F1F1;
            border-radius: 5px;
            padding: 1px;
        }
        .battery::before {
            content: '';
            position: absolute;
            height: 13px;
            width: 3px;
            background: #F1F1F1;
            left: 44px;
            top: 50%;
            transform: translateY(-50%);
            border-radius: 0 3px 3px 0;
        }
        .part {
            background: #0F0;
            top: 1px;
            left: 1px;
            bottom: 1px;
            border-radius: 3px;
        }

        .wave {
            display: inline-block;
            border: 15px solid transparent;
            border-top-color: gray;
            border-radius: 50%;
            border-style: solid;
            margin: 7px;
        }

        .waveStrength-3 .wv4.wave,
        .waveStrength-2 .wv4.wave,
        .waveStrength-2 .wv3.wave,
        .waveStrength-1 .wv4.wave,
        .waveStrength-1 .wv3.wave,
        .waveStrength-1 .wv2.wave,
        .waveStrength-0 .wv4.wave,
        .waveStrength-0 .wv3.wave,
        .waveStrength-0 .wv2.wave, 
        .waveStrength-0 .wv1.wave {
            border-top-color: #ECE5E5;
        }
        
        #card {
            background-color: #0F0;
            line-height: 26px;
            background-color: black;
            display: flex;
            padding-bottom: 0px;
        }
        #inner-card {
            width: 100%;
            text-align: center; 
            border: 2px solid white;
            border-radius: 10px;
        }
        
        .icon{
            width: 100%;
            justify-content: space-around;
            display: flex;
            align-items: center;
        }

        .radar{
            text-align: center;
            display: flex;
            height: 100px;
        }
        .chart{
            padding-bottom: 40px;
            width: 100%;
            border: 2px solid white;
            border-radius: 10px;
            margin-top: 10px;
        }
        

*{
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}



.charts-container{
    padding: 10px;
    margin: auto;
    width: 80%;
    height: 219px;
}


.graph{
    position: relative;
    width: 100%;
    height: 100%;
    margin-top: 10px;
}



.graph > .title{
    position: absolute;
    text-transform: uppercase;
    left: 50%;
    top: -9%;
    translate: -50% 9%;
    font-family: 'Cairo', sans-serif;

}


.x{
    position: absolute;
    width: 100%;
    height: 4%;
    bottom: -4%;
    border-top: 2px solid;
}


.x .title{
    position: absolute;
    bottom: -7px;
    left: 50%;
    transform: translate(-50%, 100%);
    text-transform: capitalize;
    font-family: 'Cairo', sans-serif;

}


.x .labels{
    position: relative;
    width: 100%;
    height: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
}

/* Label */
.x .labels .label{
    position: relative;
    width: 100%;
    height: 100%;
}

.x .labels .label span{
    position: absolute;
    width: fit-content;
    left: 85%;
    text-transform: uppercase;
}

/* Dash */
.x .labels .label::before{
    content: "";
    position: absolute;
    width: 2%;
    height: 40%;
    background: black;
    right: -1%;
    top: -42%;
}



.y{
    position: absolute;
    /* 2px: border height */
    left: calc(-4% + 2px);
    width: 4%;
    height: 100%;
    border-right: 2px solid;
}


.y .title{
    position: absolute;
    top: 50%;
    text-transform: capitalize;
    writing-mode: vertical-rl;
    rotate: 180deg;
    translate: 0 -50%;
    left: -185%;
    font-family: 'Cairo', sans-serif;

}


.y .labels{
    position: relative;

    width: 100%;
    height: 100%;

    display: flex;
    flex-direction: column-reverse;
    align-items: center;
    justify-content: center;


}

/* Label */
.y .labels .label{
    position: relative;

    width: 100%;
    height: 100%;
}

.y .labels .label span{
    position: absolute;
    top: -8%;
    left: -50%;
    font-weight: bold;
    text-transform: uppercase;
    z-index: 12345;
}

/* Dash */
.y .labels .label::before{
    content: "";
    position: absolute;
    width: 40%;
    height: 2%;
    background: black;
    /* width + height */
    right: -42%;
    /* height */
    top: -1%;
}


.points{
    width: 100%;
    height: 100%;
    position: relative;
}



.points .plot{
    position: absolute;
    width: 4px;
    height: 4px;
    transform-origin: 0;
    clip-path: circle(50% at 50% 50%);
}

.points .diagonal-line{
    position: absolute;
    height: 2px;
    transform-origin: 0;
    border-radius: 15px;
}
.toggle{
  position: relative;
  height: 12px;
  width: 50px;
  cursor: pointer;
  border-radius: 25px;
  background-color: #fff;
  box-shadow: 0 5px 10px rgba(0, 0, 0, 0.1);
}
.toggle::before{
  content: "";
  position: absolute;
  left: 0;
  top: 50%;
  transform: translateY(-50%);
  height: 30px;
  width: 30px;
  border-radius: 50%;
  background-color: #7d2ae8;
  transition: all 0.5s cubic-bezier(0.68, -0.55, 0.265, 1.55);
  box-shadow: 0 5px 10px rgba(0, 0, 0, 0.1);
}
.toggle.active::before{
  left: calc(100% - 30px);
  background-color: #fff;
  border-color: #7d2ae8;
}
.wrapper{
    width: 100%;
    margin-left: 262px;
}

@keyframes animate {
    0%{
        scale: 0.2;
    }
    100%{
        scale: 1;
    }
}
    </style>
</head>

    <div id="card">
           <div id = "inner-card">
                <div class="icon">
                    <!-- ícone Bob -->
                    <svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="100" height="119" viewBox="0 0 136 119" fill="none">
                        <rect width="136" height="119" fill="url(#pattern0)"/>
                        <defs>
                            <pattern id="pattern0" patternContentUnits="objectBoundingBox" width="1" height="1">
                                    <use xlink:href="#image0_1_2" transform="matrix(0.00277778 0 0 0.0031746 0 -0.293651)"/>
                            </pattern>
 <image id="image0_1_2" width="360" height="500" xlink:href="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWgAAAH0CAYAAAAQQbftAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAC00SURBVHgB7d17sJx1fcfx7+65JCE3IBcTEkJCjAEMgVxqkIiAIFaiqGgtIna0ttNOZ/pHO9N/OtMZ/u4fnWk7nba2Yy+iKKBguXgLEEANIvc7BhAkMRC5JBHJuW9/n+e3v5Pn7Nk9ey579nz37Ps1s3OSk2T32d2cz/Pd7+/yFEpbtpQMAOBO0QAALhHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0ATnUa0GqKoa6YP99sxQobWLvWbP1661y1yqyry0pHj9rQSy+Z7dtnHa+8YvbWW2Z9fWalkgGthoA2tIxCweyUU8x27jT7wAfMNmywUgjq/vD9Ugjnjo4O6+3ttdLgoM1RIB86ZPbYY2Z79sSvb79tQCsplLZsobSAf3PnxmC++mqzTZuyajlRKPf394ciuWSdnZ3hr84NWV6If6igPnzY7I47zG64wWz/fqpptAwCGv4tXWq2a5fZpz5ltnp1bHHkDA0N2TvvvJOF9PxQUXd3d4/89wrkEOL28MNm119v9uCD8feAc7Q44JvC+ZprzK68MvadU2VcpqpZFbTaG6qeBwYGQn4Xs18P079RBb5jh9m8eUp0s717DfCOWRzwS2F6ySVmH/6w2YIFNcNZVDXPmTMnC+q+MCiooB4l/Jlt3hwDf9u2UZU44A3/Q+GTwlgDgaqcV6wY9ccpnPU1BbP6zvq1qucxQ3rLFrOPfKTq/QKeENDwac0as4suyqbQVaucFcD6qsq5WFEJd+VmdFQNabU7tm+P1XTFfQOeENDwR1XumWeabdw4qg2RKufBwcEsnBXElVIlraCuWUlrsPF97zNbudIArwho+KPe8+mnjwrPfOWc2hpjUUCruq5aSWsQUVX01q1U0XCLgIY/WoyigFYromw8lXOlfCWtKXijQnrZsvg4GoAEHCKg4Y/6z6edNlzZ5itnLUIZTzgnCmkFtL6OqqRVRS9ZYnbiiQZ4REDDH1W2utnoyrk4ialxNXvSuq+FC+P8asAhFqrAHwVmuFVWzsUpzFtOlbRWHaa5052qxENwW+XKQ8AJKmj4o82PzIYr5zTYN/W7rZgnHe4b8IyAhjulnh7rO3p0eLbGiGXbU6SQToOMfepJh8eyatPwAAcIaLgzeOiQld54Y3jBSaMNh3SoygeOHLGh3/3OAI8IaLhT2r/figcPNqStUYtCWtFf1FakIaQBjwhouNNx4IAN7dtnfaGy1aDedNBMjr5QpetkUPjtbw3wiICGO8UQzF0hOIdeey0bKGx0SGfhHAYJO595xrqfftoKDBbCKQIa/vT3W0eooOeGSlrSVLtGUDhn+0eH1kbXww9b4fnnDfCKgIZPL7xgxfvvt+4338wq6J6enilX0gpnLfnu0iChKmddp5AZHHCMgIZPmv72wx9axw9+YHNDqMpUKulUORe1YCUMQBbuvdfsxRcN8IyAhl+aYfGjH1lx717rDsGcln1PtJIe7jl3dFi3wlkXj73nHiW+AZ4R0PBL1XLoRdtXv2odIajnhKAthXCeSCU93HMuFq370CEr3HSTWajKjZkbaAFc1Rv+aT60drj7yEdsMNx6ly/P5kjX2zwp9ZyLIdCznvPNN5s98EAM5xL/7eEfFTT8U0vj5ZfNrrvOOu67L1v+PVSnkk5tjSzIjxyxQuhn2549ZkePEs5oGQQ0WoNCVUuyw03tCoV0rZ50CmctE1eVnc1zPnaMGRtoOQQ0WlIK38pKelQ4czkrtDACGi1LIZwqaYWy+s3ZVLpyf5pwRqsjoNHSUqWsyvlYaGNoa1KFNuGM2YCARssrledIpxswWxDQaGnDy7e7umz+/PkNWxYOeEBAo2WlRShpA/7U3lAVrdCmmkarI6DRklLlXNlzTj1pXctwOrYqBZqJgEbLGSiHb6qcKwcEU2grnKmk0coIaLQUVcb945jnnK+kG7mfNNBMjbtcMtAEqp4L5ZWE9abSpauB94dKWi2RLgNaCwGNlqLZGjaBRSgK6WIIc5uGq4MD042ARkvRhvs2wUUo2Y53Rbp5aD38rwUAp6igMTGhvWALFpgtXhy/qn0wxepUw3cazFNd3JFrRQyWr7atCni4pbFqlU3YvHlm7353vEKLWTa7Q4OG+ftN39Pvi1OttjUgqZ3ztIPe0aNWOnIk+3WBKX+YIDbsx5hKCsz5881OOcUG16+30oYN1rF2rRWXLDE74QQ1hSfcchj1GOHW19ubDeRlC07CfQ6FcO4pXwllxIDgwoVmixbZhCgstQ+0AtNseAc8BXHW0w56yzM95tS5CMD4nlAp7mEdjl8XB+g/eNCGXnjBOvfts46XXjJ74434Z8wsQR0ENKpTSC1bZgNnn21D27db6YwzbGjpUusM4dgZKlLNpJhqMOelvZ1VNWtgTyGa5jkXp6F/nLYl1WPosfV4c+fOHZ750TDaH0SP9fbbWSXdsX9/djXxjocesoIu56U9roEaCGiMpip169Z4ialzzrFjoVJWs2FeCGYF5nRRUGofDQWnHkeVc3EaB/fSDniicE7V9HTQCUCPpYUz83TiefVVK9x9t9nu3fFqMeV2DpBHDxrHqXoM7Qu77DKziy+20urVNqgqWZVmCBhVtwqa6QrN1AdWME9X5ZynalnBrBNCvgc9HdKSc/XYh8LjlkK7qLBihdmmTWZ33GG2d6+ZetVADgGNSMF01llm11xjtnNnNrA2EKo93VIlq+pWbYjpqGwHK5ZvF5s0LU5Vsx5TIZ0q90aHdGqn6LF0UsheR72u4ZNK8fzz4wVx3/Uus9tvN3v9dQOSjmtXrrzW0N5CFWnnnWf2hS+Y7dhhpfD7/GZEChYFpqq/fBXdqCBLy7GL5QHBYpPnLKfnouesKlrPs1HPrfISXOl1HCp/IinqsTTouW5dfB9+/es4oAkYFTQ0SyP0mbPKedu27PeDIVQULCmcR+wUF77Xe/iw9YeP491h4KugQa4p9k8Hw4mgGIIxOxHM0JVQ9IOgwTw998HwHKc0WKjnoM2aQoU8EG6dJ51kXbnKPF2SS58YUtVeVAX98Y/HGSc33WT2m98YQEC3M1WqGzeaffKTZmefbaXw+6ytEUJCYTwczuqfhjDWYFbHE0/Y3CeftNL+/dkUMvWnpzpdrLPcn2125TzqOMKtQ31o/WaqJ4oQ8IVQEXcvWWIF9Zs3b44tpOXLs7nk6dNCvrWSTV1USGvg8rvfNXvrLUN7YxZHO9OiD7U1PvaxbE6zWhr5tkYWzqroXnjB7M47zX78YzMFswKEWQfjo5OOPqWceGIM6Esvje2kk0/OTgJpTrZkIa1fPPec2X/8h9k99zBXus3Rg25X6ndefLHZrl1WCpXbQLmtoWAeHihThaxQ/trXYkCrP9qAirmtpEUr77xj9sorZs8+G+c+q1pevNgK5X63+tG6dajyVpgr2H/1K6roNkdAtystfb7iiqy1MVDe2H5E5ayBKoXyN75hFtoa1tNjmCKFtVpFzz8fVxOq77x0qRXD6z5i4FDvgabgKdRVTWvVIdoSmyW1I1XPmq2xdav1l6/fp3AerpxV4d11l9n118eKL/w5GiSF9H33md18s9kvf5l9Ly07Tysqh7S8XoO3OpHO0MApZh4B3Y7CD31p2zYbWLAgC+c0IJhRgDz1lNn3vjccHpgGqo737DG77Taz117LvqX3Ie07ki2eOe20GNCdjOW3KwK63SiI3/MeG1yzpupFV+3QoRgcamswEDi91EbSa61VhOU2Rqqk1e7oDSfQofBeqQ2C9kRAt5mSqubQ3+w74YSRlbNoAFCDgvfeS8+5WV591eyRR8wOHhz+1nAlHb72b9xoQ2eeaWhPBHSbGVq0yAaWLbPOhQtHX9dP1fPjj8fQQHOov//kk3EwMNdOSifPwVNPtb7waafUxRUV2xEB3WZKy5db5+rV2cq2kX9QitPoDhyI08LQPHrNFdAaPMzJVm6GTzrF8ImnpKl3aDsEdLvRnOfFi7M5z6NoeTFLjJtPVbQGZBXUOUPl6Y+D2v6VgG5LBHSb6TjpJCuGH/a0arCUPlbrqy4JxZaXM0NtpVxrKYWzdOk9m+hVZDArENBtphA+MneHClofn7PqLD9TQ1O/dEPz6cRYPjlmMzjKlwDLtijVnGhd+xFthwmW7UZLi7UoxWx4ox5V0Z3aclNtD6bWzQzNoAmhXMpVzhrEzXbV014euYvpon1QQbcbzdoIt7Qx/nAlnXrSLEyZGeXrIuritfpUkzb3z5TfM7QfArqNVYZ01YFDNEWpvOReIZ1d2ZzVgzBaHG0vhXRfCIYhptfNKF1dpYNwRg7/EzAc0sZiiBmj96CLXjMqENDI0OF0gD4zKtCDBgCnqKBnG82XPekks5Ur400LHNTTTNWZro3XDHrMxYvNtPG8jkOXeNL3NM9ae35ocyCtWtTvPfa+daxavZeOX69p/vi1LF7Hr8t/Tffx67XT1W+0/ajo8bSZlTb913HoeHT1G6ZIzjoE9GyhSyTpCh3ve5/ZBz9odsYZI8M5BfR0X5hVj6Ng02bzOg591XaZ6nGnC9Aq5HQ5p5/9LO6cp2selq/LN+N0jArjc881u/DC7Ioz2fGrP5+OXxc0ePlls/vvj8evZdrTeVEDXWh2167jUyDTZbS0RalOdA89FI/j6afjsWHWIKBnA10hRZWxLv6aLkg6E1fI1gDX6aebXX55rPhOOaX6ZvM6Xh2j9jrevt3s9tvjFUbefHNm52HrWLVBvsLwooti9Vxt0C5//Nu2xeP/6U/j9QOn4/h1Yqj1OupTyvr18XXURRZ0mTKFNjNyZgUCutWpMlXVfPXVsVqdqSXBCjLtW/y5z5ldcIGZlifXG/Q64QSzLVti5a+q+9Zb48f2maDjf+974/Hv3BmPbazj158tWGD2e78Xg1yfVu64I+5n0myq7jdsiK+jjuVb34qfUFh01PIYJGxlIVRKIZRLV10VK7mZ3K/h1FPNrrwyVp4KrvHOSFClv2qVla64woZCS6E0b57NhFLo75Y+/enY1hjPySXR8a9ZE5+7/u0MHX92HOEkV7r0UhsKt5Iqa7Q8ArqFlZYts75Q7fXpqhszOUVLH7Xf//54068nSHWervLSe8klNqhPAc2eCxyOuT+c4HpD33loMotE9NqvXRtbO+pZF2fux2owBHO/Xke9F8xrb3kEdAsrbNpkHTt22GConLOLjM5U3/Gss2L1qMGsCZ4otMRZS8z7BwetGAY2O8PzafrexyFUOxRqYXCwJwy8DU5mNoRCOZwobetWM+3fPAP0OvZpsHLdOuvQ+6FPNWhpBHSrUo9UgRYG5bTrmYJuRkJaH+nVR1b/eRLhrP0ndNxa3tytPq4G6TS42Cw6/lA9d4RB1rnl6r93siGt1o6OX33gJsvCObyO6aKzBQ3WavCQxS8tjYBuVZr6pfm54Ydx+Pp1IVQULk0NaU1JCz3kyfReU6hkl3YKg526SGpW9a1ebU0T2kTZ44WTXLpYq0zqdVQVrVaHetJNpPddJ7p0/AppXTkn+0TD0vGWRkC3Kg0C5QaCVIGmcEm7ok2EqtnSJEb9S+HjfEmV7wT6rqna100nlhEXr9X0Nd2aRa+hWirlx8+HdE9Pz4Qr6VI4ceo2Gdl7YBOjk5xOJtleHqqc0+uoT1iq6Iv8iLcy3r1WpRCpmLWhkNYPaapMxxu4+nvah7hvEostNKjWH0JtcALhPqpyzn8MV7Do1ixqa1QMbKaQzl6XCVTSWcsmvP79lRfkHee/Hcjvyz0OOnmk13G4ck50DGlxEFoWAd2qFIhVQjFV0qlKrRfS+d51xyQ+Ditch8qtlXrVZr7nPKpyPv6Xmjt/t8bjTTSk03NTwE72hyrdh05g9d63VDkP95xn+nXEtCCgW5WW+epWRb6S1g9xrR/2FM4K1myT+EkEdDH8+znhfjpCQKT7qiUdT9XKOWn2dRG1p4VuVeh11MBhvZBOr6PCtVMVbY33ZSxZi0KvSQjceie7qj3nSvo0pOXzhHRLI6BblZYV61ZDqqQVKtUq6Xw4Z5dXmuxg0tGjVjh8OO5lbNVnQIyrck5efz3emiW9jjWCLJ1M9DqqJ10Z0sOVc/kk1/XGG1bQ5kWToNckXYewViWdBoJH9Zwrvf129t6w5Lu1EdCtSiGmnczG2GQom7oWQiP9UKcf9srKuWsqCxq0f8Yrr1jx2LHhaq6yks5/HK9ZOYvCREuU9++3ptHrGI4/25WuhlRJS76SzlfO2UlOlexLL8XnMEl6ZdJrVHmyS+9jeh2LYw0Aasm8ThTscNfSCOhWpY/RzzwTd4Ib42NsCulUSaevoy5MOlk6QTzyiNnjj1sxHEcKl/QY+cpZIVcYa9BKVd++ffHE0yx6HR94wOzRR8esNlMlreeUZneMqJxVzR45YvaLX5i99ppNRbVKuvIkN2Y4a6BRx/H887Q4WhwB3cq0vaTCRcE2hnwl/U7o7zakcs579lmzPXuyXdSK5ctnKWT0WMdCZVq3chZVejrh6Pnoo3kz6XXU8StY65zs5pXne+u5DVfOOsnp+HU/2vqzzvsxHoXc66gTgm7jCmedZBTM2n70wAFDayOgW5k+nt9zT6xg6+ynrApQP+z95al0HY1cwKD71N7O2jI0hJNCWvevaj0fNDUp3LSn8m23mT32WPP7pjp+nRi0ZagGKMcIaYWjbmmuefa6plDUtqNPPdWwqjX1mdVK0WPpRDBmOOtxdREBHcfevbQ3ZoGOa1euvNbQuhTS+qHUggttN1mlKs5Puesuz9EdDpcUnPrhVvX38MM2KdooXj3PcP/9y5ZZb7jfFCiq2FOwjaKP4+rb3nij2d13N3cGR56uSKIKWnPLtdRcXytOKpUDgtn3NJsinFwKN9wQT5Y1ZoTUpcpcezpry9Py4+Z7znqv9Hu9X1VfR4WxXn+Fs7Y91dgAWh4B3epUvb36ahzoUmhoiW8Kl3BL4awepqqxVM2mGQLDIT3VgNa/D6FQCj3k/hCyHStW2NyTT86mjg2FP1O4jHgs3dQKCL1ru+46s927Z/ZqIDoeDaypp6/XVEvAFZoKw/LrqHBObY054VYMz7OkTy9f/7oVFc5TOblUBHQaK1AYq3evk52+p/dtxMlOx60+uir4r33N7OabCedZhA37ZwMFivq3//qv8aOttprUJa9CQA6EH2T9YCtQOlVl6aNybkl4CpxGrTcrhCp0zi23mD33nBV0HJs3W7cq6hDOveFY5ihcFMyqmh98MLZGNOthAivoppUGKHXC0Inq/PPNNm3KPpkMhIDMKufwV7oUxKFq7vz5z61Dx69ZJw1sJ+Sn0mUn1PLlrbrLy+N7Q7U/J7yeHZp5osfWSUKX39LJZYzZKGg9BPRsoUpK7Q5VohrwKm9iVAytD/2QD/ecL7kku6kiG9H6CL9vWEirXaCeriryRYusqGo6hPSgLjCgP9NHcbVl1A7wNstAx6NBSgVeCODsSikrV1ohvJ6dOskpnPWJpXz8hQYff6qcRyzm0WDfnXdaMQzGarKfloSX9GlD77faMvo1851nJQJ6NlI1qgAJt1FDgboytELabHihw3AlbdawkM6oqiwvBCmECr/l/rPljr8Zx57traF2ReUiFJ0wVNFrZobxQ9tOeK/bXJoH3aePxqFym/g2P2iE1OPWUu8xV1qirRDQiO0ODTCyNeWM6tYMnHpTEtFWCGhkulRJd/LfYaYolAvlGSNAQskEeEE4owIBDQBOEdDtZqyN3MuLWzADxnrd2Xy/bRHQ7UZT8KotClFAaJCKPvTM0OIhbWlaLahrvWeY9QjodqPpdLVWmzX7eoA4Theu1a2aMa6eg9mNgG432rNYt0qq3LT6sFZIYHqtXBlv1Wj1pW5oOwR0u6l1SSkF9PLlcZMgNJfaSqefbnbqqdX//PDheEPbIaDbjfbB0F4S1fZu0Daba9bQh262devi5lbly2qNoP1K9H4R0G2JgG43+kE/eLD61piqoDdvNluxwtAkWsF59tlmGzdW/3Pt+qfLgJUvtID2QkC3G231qb2jq10RXJvCn3++DVx8sZUYLGyKoVWrbPCcc6y0dOnoP9TMjSeeiJfSQlsioNuNfui1d3SVC4pqN7Vj4WN2n/Zx3rIlC2xMo0WLrPShD1nv9u3WG96LUuVcZ+1I+Nxz1U+maAsEdDt68cW4WX7uBz9tEq99iOede64Vdu0y27CBkJ4uCxaYffSj1vGJT1jnkiXZTnZ6/YdDWl/1Pqm9wRzotkVAtyP9wOvKK7qVr7Gnq28rHLKtLjVYddFFZl/+cuyPMmjYWArnyy4z+8xnsoFZbcyvq4Vn+0GXL6CQTat79NF4lRS0La5J2K50FY4QxkPr1llvCOSiKucQEsPXulMoa9qXZnZo3rSu18dA1dTo00joOdsf/qHZ5z9vtnr18MrB9LpnAa1PMrt3W+Gmm7i+YJsjoNuVptmFFkdvGAwsrV9vcxcuHH21aAWKAjoMYmVXDNcKRFV2qsDZG2J8FMA62en1C/1m++IX41ctCKpY1q32UlEXqH38cSt8+9vZJa7Q3gqlLVv4SWtnquIUGuo5ly8mW1W6Vp8uvaRrHio8VN2Fim/kX4v/nfKbzucHvxq5GX2t+612DI18vHE9N53sNBNGJ7gdO8wuuCDOd+7uHusB4sCgrs79ne9wAVgQ0G1PoaI+85e+ZLZzZ/1+s0JErQ61SBTYCpHcFa3VR9WAl6pBXVcvuzCtLnKqC9OG3xcbeNUW3WdveY8K9XF13wOhuu8PN10lpqvBvXM9r4Hw/PS8OsPz0wVee8MJSsE8J38lFH1VEM+fbxY+mYTeUf3BVr2ump/+zW+a/d//xdcWbY/Rn3anYNBc2698JQbvhRfGXe1qSeGjm/buqFAM91cIodWrUA5/RyGq0NTgY7HBgak47FZIhpDuC8elk4J+rXDu1KeBBlfQnXouPT3WE0J6bujbD5RbPd0aWJ3KiUfvgT6N3HKL2a23Es4YRgWN4zSt7qqrYo900aIpBZwGu3pCmKmqVJh1jRX6U5RmoQyWg7N7Gq/rl30iKD83nQj0eB1TmYqokNec9BtvNLvzTsIZI1BB4zgFxT/9U1wc8dnPmq1dO+kLySrI1AJQUJameUCxVF7kMV2952qPp+c25eelfTY01fH6683CwGBlPx+ggsZoal+cdZbZ5ZebnXdenIGg9sQ4gi9VmPr4n3rQ+rWq2umootPcYfWfU4tDX7P53A0O6qFyOyXrp4fno8fW9/TrcVfRmj2jfVA0v/n73ze76644MAhUQUCjNs1C0DaYmoGgmyrqOr3dFM4KLbUAFGYaXNOt0SGtx6kM5HxgN7LVkQYk02Ie3b9+rVZHaq101uqxq9JWMKt9oUp59+44G0bBzCpBjIGARn0KOYW1FlkopLVntH6vQMq1QBRUCk0F5oiwKlfRqjY7GzSTI1Xmit/Oiup+qHwcxcrjmILh4w/3N+L4y8eh49FzGz4hpFBW20LBfOBA3JlOe3Gz4AfjREADgFPsxQEAThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0AThHQAOAUAQ0ATnUaIPPnm11+udmVV5oNDprdeKPZ979v1ttrmGFnnWV29dVmmzebPfig2fXXm+3bZ5j9COhWcdJJ8Qd07Vqzri6bNIXvgQNmjz1m9tprx7+/Y4fZZz5jtmFD/L1+/cYbZj/+sc0KJ598/PXrrPPfXq/RsWNmr75q9uyz8XUqlWxGnHii2a5dZr//++HzbvjAu3Kl2eHD8T185x3D7EZAt4JVq8z+5E/MPvYxs44OmzKFzZ49Zl/5itlzz8XvKcB0S/TrJUtsVpjK66cQfOghs+uui9Vrs4Na78GyZTGcRcev3+v9IaBnPXrQrUBV7RlnNCacpVCI9/nud4/8nm75388Weu3e+97JvX4nnGB2wQVmf/d3MeCb/boomIvF+t/DrMS73ArefjveGkkf4XVrB3PmxNtUrF5t9vGPm23dakCz0OJoBeoX33RT7J2uWTO6h6pqSgGU/776qBrg09e8oaHYU/3Od8weeMAwAWeeGW9qeQBNQEC3gv5+sx/8IN6qOeccsz/7M7Pzzjv+vWeeMfv3fzf7yU8MVfzwh2Zf/WocbMtTG+Rd74qDchqcW778+J9ppotOkOoLawAVmGYENMZHoXTuubGCVFD97ndxgPGRR8xef90mTAN327ebnX567PNqwOuFF+JA3K9/bdNOnyx6euLzqHT0qNn+/fHE+Ed/ZDZ37vE/W7gw3sYT0BrI27Qp9sCXLo3967feMnv++dGzaCZLn5zUX1frRY8xMGD2q1+ZPfyw2S9/OfoTFFoKAY2xLVoU50dfdZXZqaeOHiR75ZU4w+GOO6qHXZ7+7dlnm33uc2bnnx+DrtKRI2b33BPn+qYZJjNB/Xk9N5181H9OVGHXG2xcv97s0582u+SSOOOiGj1Pfbr51rfMnnxy4rNDdNL48IfNrrkmnjQr2146+ehkl2afqLWFlkNAozYF01/+Zfyon68i8xTamsIm6mvXqthUJX/iE2Zf+ILZihVW0+LFZldcEUPnP//T7K67Zq4KVKhVBttvfxtv1XR3m112mdmXvmS2bp2NSc9TJz59ivjGN8xuvrn2/VbSie2LXzTbubP6SU70fn3gA2YbN8aQ1ntT7wQKdwhoVKc2hgJVAVBvBoSqxAsvjIs6nnhi9J/Pm2f22c/GdoEWXoyHpgH+9V/HCv6WW5of0nr+Ctl8D1qtD7VhfvOb0X9f4axVmArnWlVzNbr/P/3T+DorSFVZ1/P+98ev45k2qGPRMenTi6p1Voa2FAIa1aVeq2iK30svxZV1qoT1EV7Bkm93vOc98VYtoC++2OxTnxoZzmk2ie5X968et/rR+b+jwTr9u0OHzO67zxpKJx09Zl/fyO/rOem5aZBQLYT0yUEtCM3e0K1aO+Kii8z+4A9Gh7NOLAcPmr38cnzO6r3rlj/p6WSg5/nmm2Y33FD/ZJQPZvWcNdCp+9e/06cefarJf+LRa6opgurt795taB0ENGpToOzdG1sNjz9+PJg0k0FtjY9+9HhYKOwUDGnAL1El/KEPxT9LNIj19a+bfe97I+d3K6BVZV96abwfUatDrQD1gxXmjaJj0m089Dr8/OfxmKv1xXXC0jFXtjX0iUJVsVZtptdEy/TV1lCLYtu24wtONKCo+1CFPt7pjzqW//1fs7vvjj1n0fuh+//yl+P9p5OoXtsPfjDO7qmcuQK3WKiC2jRD43/+J844yFeNCth77x0ZmAoCzSKoXB6uKYC6JarC//u/47zuysU3L75o9s//HFsaqbLV/W7ZEm8zRc/z1lurfzqQasenSvsf/iEOnuZPWJoZopPe3/+92Z13jnxdtSmS7mc8i2qeesrsX/4lnuRSOIuq6J/9zOwf/3HkFMs0QKsbWgYBjer00Vkho3CuRlO4dMvLt0VEYa22Rz60VYmqQqw1a0HT137605GVqloOmqqW3yukmVR9/u3fmv3N34yc0SE6KWkgLv8cdRK67bY4e6IWVcoK11/84vj3FMz6xKDHG4t64ZoTr6CvRQGuud75alnHrip/KpttoalocaA67ZimwTBVfNVUW36uH/z8D78CIR9oCn0NGOoj+FjSnOG81LtVn3YmpFkoen5a4KKWi6h1k2/fyNNPx1s9ahvpppNYer7arU43tSJq0clL91+vV52OQ6+bqP1xyimxt6953nCPgEZ1ClPdatFsgMoZAZWb+GhwKj/op7m66rPqNlGazaFbo2jusSp1VaOVNMCmkNT2pKo40xxjhbOOXS0e9ZZ18qp8jvpkoOpY7Zp6dLJR+0TT69Jzq7y/ajTYN57FPAphnUh0TOkEkO6fgG4JBDQap3JHvEZsUlTrvqdKwaX+sMK2lgUL4qKaz3/+eIBqxoV66vffH6tcTa/TLdECF32yGO+0QPWnNT853X/l/VUz1lzsPPXxdSz6mt6HRr4nmHb0oDF9GrUtpsJOvVpVps2kcFMQq9rO00KbWottJvqcddLJ/31Vu/VWFU7kMSpPbNUW38AtKmhMH1Vu+XnG+vWjj8YBrPFSG0F7V2hviZnoP2vvDN3y1I9O0wA1gyI/i0LVqaphfR3PohBV5Lol1VpHlbQKUbd6VbTuV1fiyVfk47l/uEFAY/poVVx+ZZyCQgsqNJ94pgb7JqpayyFf5VYGeP5iCPVORKrC9ffUSkk0OKvbWDQPXbd6feTTTou3PL3urfLagxYHppGmeFUuitCua6206b1WBlauDsxXzeph66STb0toBzvt/Ffv2ofV5k8rdOsFr04A6oOnKr4azdjQ39GxJKqcdbyN2EUPTUFAY/qki67mK8K005sCbCyaPaFpbdr1Ll9hNpOm+2n1nRaQ5OWrXH3VtDc910SzJHTsWuJea2BTq/y0vFuVcKIBQ1XdlfPLK2mWiVZXail6rf04tFGSlnerxZHoRKJZIzN1AVxMGC0OTC8tdNFNmykluoK4Fp9oBzctptBeG6KBLy3S+OQnY/ioctVAnVYeqi3SyN6p5gbrMaptTqT+sdoPClFVq5UhqCo0P/sj7dGhXf9SIKt18Vd/Fec4f/e7x6tihb52vNNJqnJpuF4n9drHMwNEx/8XfxEDXjvVpU8qun8twdf958NfA4O6/1qrIeESAY3ppQpaIawwygeGfq/d6v78z2NPVIOBafArH4iqnteujcHTyD0kNMdZt4lSFaowzl+kQOGbnqM2z08U8toTQ3s2q9JWSKq61mKdSnpuWlmohSvjlXaq01RA9cEV7KqY84OOiRasaHn+ZC6ugBlDQM8G1aZOjWe61lRUu/9aj6k9nRW82iAov32n5GdEVKMKWh/5U5XdqGOdDIXs7bdXv4yYvqeTibYOrayMVZFr9V4tCs1vfnP8O82l55Kq9bSwphYtalGVPdbScLhEQM8G1Ubm9UM/kWopzUZI+11Um16WpxV4WgqutkNa+KB9NKpdCkp/59vfjn/2x388cmnzWBTK2n5TmyfVWnI+Hum55Cv4iUirA9Vm+dGPRm5+lP872h9DIa6d/tRjrzdXWf9m3z6z//qveBKr9hwrp/GJdq/T42g3vnqrDrWiUUvTddzMf245HdeuXHmtobWllWUKV1WqWlihS0ZpzvF4KczVWlCI6YdfgaptMscKRoW0ql9Vh1pVd+ONoxd1JAoHhYUqTQW7PoprvnBlf1cf0/VxX7vHaUc27fim1XlTocfT/eq56THrBaf+rh5Tlad2hlMP/N/+Lb6e9U4UanfoOerf6jnq/aiczaEl9Hot9B5pRzrdb63w1CcIva/aLEqtC702em8U6nqt9fqr/VO5OlDPWa+hdgfUcyCcW1KhtGULQ7povnT1bA0KKmAUmhqwU3ClC7a2Oj0nPUcNGKpfnC4aq2pcQTvWXid5eq00WKk5zfq3uuVbNrpvPUZqH2kAU4t7xnsJLbhFQAOAU8yDBgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcIqABgCnCGgAcOr/Adg0x51D9mWpAAAAAElFTkSuQmCC"/>                        </defs>
                    </svg>
                    <!-- bateria -->
                    <div id ="baterry_wrapper">
                        <div class="container" style="float: left; margin-left: 10px;margin-top: 10px;">
                            <div class="battery">
                            <div id="lbat" class="part"></div>
                            </div>
                        </div>
                        <div style="float: left; color: white; font-size: 18px; line-height: 26px; margin-left:15px;margin-top: 10px;">
                            <span id="vbat">0</span> Volts
                        </div>
                    </div>
                </div>
                <div class="wrapper">
                    <div class="toggle"></div>

                </div>

                <div class = "radar">
                    <div id="radar" class="waveStrength-0">
                        <div class="wv4 wave">
                            <div class="wv3 wave">
                                <div class="wv2 wave">
                                    <div class="wv1 wave">
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div style="color: gray;"><span id="distance">200</span> cm</div>
                </div> 

        </div>

    </div>
 
    <div class="chart">
        <div class="charts-container">
        </div>
    </div>


    <div style="color:rgb(128, 128, 128); font-size: medium; text-align: left; width: 300px; border: 0px solid red; position: absolute; top: 0px; left: 0px; visibility: hidden;">
        DEBUG: Vel: <span id="speed">0</span>% | 
        Ang: <span id="angle">0</span> | 
        Botão: <span id="button">0</span>
    </div>

    <div style="display: table; width:100%; height: calc(100% - 80px); border: 0px solid green;">
        <div style="display: table-cell; vertical-align: middle;">
            <div style="display: flex; align-items: center; justify-content: space-evenly; align-content: center; flex-direction: row; flex-wrap: wrap;">

                <canvas id="canvas_joystick" style=" margin-bottom: 440px;"></canvas>
            </div>
        </div>
    </div>

    <script>
        
    const toggleBtn = document.querySelector(".toggle");
    toggleBtn.addEventListener("click", () => {
        let active = toggleBtn.classList.toggle("active")
        var data = {"active":active};
        data = JSON.stringify(data);
        console.log('Button On: ', data);
        connection.send(data);
          
    });

     class LineChart {
            constructor(config){
                this.config = config;
                this.intervalsNb = config.y.values.length;
                this.intervalBounds = 100 / this.intervalsNb;
                this.intervalMeasure = (config.y.values[1] - config.y.values[0]);       
                this.intervals = [];
            
                this.drawChart();
            }
            updatePoints(newPoints) {
                // Atualize os pontos existentes
                let pointsContainer = document.querySelector(".points");
                let plots = pointsContainer.querySelectorAll(".plot");
                let lines = pointsContainer.querySelectorAll(".diagonal-line");
                 pointsContainer.innerHTML = '';
                lines.forEach(line => {
                    let parentPoint = line.parentNode;
                    parentPoint.removeChild(line);
                });
                newPoints.forEach((point, index) => {
                    let [x, y] = point;
                    let plot = document.createElement("div");
                    plot.classList.add("plot");
                    plot.style.backgroundColor = config.options.colors.points;

                    // Calcular as coordenadas (x, y)
                    let _x = ((index + 1) * this.intervalBounds);
                    let _y = this.inputPosition(y, this.intervalBounds, this.intervalMeasure);
                    plot.style.left = "calc(" + _x + "% - 2px)";
                    plot.style.bottom = "calc(" + _y + "% - 2px)";

                    // Adicionar o ponto ao contêiner de pontos
                    pointsContainer.appendChild(plot);
                });
            }
            

            drawChart(){

                try{
                    this.checkConstraints();
                }catch(e){
                    console.error(e);
                    return;
                }

                let container = document.querySelector(".charts-container");
                
                // create 'n' intervals
                for(let i = 0; i < this.intervalsNb; i++){
                    if(i == 0){
                        this.intervals[0] = [0, config.y.values[0]];
                    }else{
                        this.intervals[i] = [config.y.values[i - 1] + 1, config.y.values[i]];
                    }
                }


                container = document.querySelector(".charts-container");
                let parent = document.createElement("div");
                parent.classList.add("graph");  
                parent.style.backgroundColor = config.options.colors.graph;
                container.appendChild(parent);

                // Chart Title
                let title = document.createElement("h2");
                title.classList.add("title");
                title.innerText = config.title;
                title.style.color = config.options.colors.text;
                parent.appendChild(title);
                
                // Axes Container
                let axesContainer = document.createElement("div");
                axesContainer.classList.add("axes");
                parent.appendChild(axesContainer);
                
                // x-axis
                let xAxis = document.createElement("div");
                xAxis.classList.add("x");
                xAxis.style.borderColor = config.options.colors.axes;
                axesContainer.appendChild(xAxis);

                // x-axis label
                let xTitle = document.createElement("span");
                xTitle.classList.add("title");
                xTitle.innerText = config.x.label;
                xTitle.style.color = config.options.colors.text;
                xAxis.appendChild(xTitle);

                // x-axis data Container
                let xLabels = document.createElement("div");
                xLabels.classList.add("labels");
                xAxis.appendChild(xLabels);

                /**** ./x-axis */



                // y-axis
                let yAxis = document.createElement("div");
                yAxis.classList.add("y");
                yAxis.style.borderColor = config.options.colors.axes;
                axesContainer.appendChild(yAxis);
                
                // y-axis label
                let yTitle = document.createElement("span");
                yTitle.classList.add("title");
                yTitle.innerText = config.y.label;
                yTitle.style.color = config.options.colors.text;
                yAxis.appendChild(yTitle);    
                
                

                // y-axis data Container
                let yLabels = document.createElement("div");
                yLabels.classList.add("labels");
                yAxis.appendChild(yLabels);
            
            
                let pointsContainer = document.createElement("div");
                pointsContainer.classList.add("points");
                parent.appendChild(pointsContainer);
        
                
                for(let i = 0; i < config.x.values.length; i++){
                    // insert x-axis data
                    let x = document.createElement("div");
                    x.classList.add("label");
                    x.style.flex = "calc(100% / " + config.x.values.length  + ")";
                    xLabels.appendChild(x);

                    let xSpan = document.createElement("span");
                    xSpan.style.color = config.options.colors.text;
                    xSpan.innerText = config.x.values[i];
                    x.appendChild(xSpan);
                
                

                    // insert y-axis data
                    let y = document.createElement("div");
                    y.classList.add("label");
                    y.style.flex = "calc(100% / " + config.x.values.length + ")";
                    yLabels.appendChild(y);
                
                    let ySpan = document.createElement("span");
                    ySpan.style.color = config.options.colors.text;
                    ySpan.innerText = config.y.values[i];
                    y.appendChild(ySpan);
                

                    // Plot Points on the graph
                    let point = document.createElement("div");
                    point.classList.add("point");
                    point.classList.add("point-" + i);
                    pointsContainer.appendChild(point);
                
                
                    let plot = document.createElement("div");
                    plot.classList.add("plot");
                    plot.style.backgroundColor = config.options.colors.points;
                    

                    // Calculate (x, y) coordinates
                    let _x = ((i + 1) * this.intervalBounds);
                    plot.style.left = "calc(" + _x + "% - 2px)";
                    let _y = this.inputPosition(config.points[i][1], this.intervalBounds, this.intervalMeasure);
                    plot.style.bottom = "calc(" + _y  + "% - 2px";
                    point.appendChild(plot);
                


                    // Create the diagonal line element
                    let dLine = document.createElement("div");
                    dLine.classList.add("diagonal-line");
                    dLine.style.backgroundColor = config.options.colors.lines;
                    point.appendChild(dLine);
                
                
                    // Calculate the next points then, Draw the "virtual" right triangle
                    let XnPlus1 = 0;
                    let YnPlus1 = 0;
                    if(i == this.config.points.length - 1){ // Last Point ==> NO Diagnoal Line Needed.
                        continue; 
                    }else{
                        XnPlus1 = (i + 1) * (this.intervalBounds);                
                        YnPlus1 = this.inputPosition(config.points[i+1][1], this.intervalBounds, this.intervalMeasure);
                    }
                    

                    // get current (x, y) coordinates
                    let Xn = i * this.intervalBounds;
                    let Yn = _y;
                    
                    // calculate the diagonal line width
                    let a = Math.abs(XnPlus1 - Xn);
                    let b = Math.abs(YnPlus1 - Yn);
                    let c = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
                    let dLineWidth = c;
                    dLine.style.width = dLineWidth + "%";
                    dLine.style.left = "calc(" + _x + "%";
                    dLine.style.bottom = "calc(" + _y + "%)";


                    // calculate the angle of rotation of the diagonal line
                    let dLineAngle =  (Math.atan(b / a)) * (180 / Math.PI);
                    if(YnPlus1 < Yn) dLineAngle = -(Math.acos(a / c)) * (180 / Math.PI);
                    dLine.style.rotate = -dLineAngle + "deg";


                    // Apply animations if configured
                    if(config.options.animated){
                        dLine.style.animation = "animate .8s ease-out forwards";
                        parent.style.animation = "animate .5s ease-in-out forwards";
                        xAxis.style.animation = "animate .3s ease-out forwards";
                        yAxis.style.animation = "animate .3s ease-out forwards";
                    }
                }
                
            }




            /**
             * Validate the rules for drawing the chart
             * Rules:
             *  1. no negative values in x
             *  2. x & y values should be equal
             *  3. distance between two y points should be equal
             * 
             * 
             * @throws Error
             */
            checkConstraints(){
                this.xValuesConstraints(this.config.x.values);
                if(this.config.x.values.length !== this.config.y.values.length) throw new Error("X & Y should be equal in length");
                this.distanceConstraint(this.config.y.values);
            }


            /**
             * Validate x-axis values constraints (no negative values allowed)
             * 
             * 
             * @param {arr} values
             * @throws TypeError 
             */
            xValuesConstraints(values){
                values.forEach(value => {
                    if(typeof(value) !== 'string') throw new TypeError("X-axis values should be non-numeric.");
                });
                
            }
            /**
             * @param {arr} values
             * @throws Error 
             */
            distanceConstraint(values){
                let distance = values[1] - values[0];
                for(let i = 0; i < values.length - 1; i++){
                    if((values[i + 1] - values[i]) !== distance) throw new Error("distance between two points should be equal");
                }
            }

            /**
             * Determine the exact vertical position of a given point
             * 
             * @param {mixed} pointY 
             * @param {mixed} intervalHeight 
             * @returns 
             */
            inputPosition(pointY, intervalHeight, intervalMeasure){
                // Negative Point
                if(pointY < 0){
                    throw new RangeError("Negative values are not supported, yet");
                }
                return ((intervalHeight * pointY) / intervalMeasure);
            }
        }    
        
        var connection = new WebSocket(`ws://${window.location.hostname}/ws`);
        let newPoints;
        connection.onopen = function () {
            console.log('Connection opened to ' + window.location.hostname);
        };
        connection.onerror = function (error) {
            console.log('WebSocket Error ' + error);
            alert('WebSocket Error #' + error);
        };
        connection.onmessage = function (e) {
            console.log('Server: ' + e.data);
            const data = JSON.parse(e.data);
            if (data["vbat"]){
                document.getElementById("vbat").innerText = (data["vbat"] / 1000).toFixed(1);
                var lbat = (data["vbat"] * 100 / 9000).toFixed(0);
                if(lbat > 100){ lbat = 100; }
                if(lbat < 2){ lbat = 2; }
                console.log("lbat=" + lbat); // debug
                document.getElementById("lbat").style.width = lbat + '%';
                if (lbat < 20){
                    document.getElementById("lbat").style.backgroundColor = "#F00";
                } else if (lbat < 70){
                    document.getElementById("lbat").style.backgroundColor = "orange";
                } else {
                    document.getElementById("lbat").style.backgroundColor = "#0F0";
                }
            }

            if (data["distancia"]){
                document.getElementById("distance").innerText = (data["distancia"]).toFixed(0);
                document.getElementById("radar").className = '';
                if (data["distancia"] < 10){
                    document.getElementById("radar").classList.add("waveStrength-4");
                    newPoints = [
                        ["10", 60],
                        ["20", 1],
                        ["30", 1],
                        ["40", 1],
                        ["50", 1],
                        ["60", 1],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
                } else if (data["distancia"] < 30){
                    document.getElementById("radar").classList.add("waveStrength-3");
                    newPoints = [
                        ["10", 40],
                        ["20", 45],
                        ["30", 50],
                        ["40", 1],
                        ["50", 1],
                        ["60", 1],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
                } else if (data["distancia"] < 60){
                    document.getElementById("radar").classList.add("waveStrength-2");
                    newPoints = [
                        ["10", 10],
                        ["20", 10],
                        ["30", 10],
                        ["40", 30],
                        ["50", 35],
                        ["60", 40],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
                } else if (data["distancia"] < 90){
                    document.getElementById("radar").classList.add("waveStrength-1");
                        newPoints = [
                            ["10", 10],
                            ["20", 20],
                            ["30", 30],
                            ["40", 35],
                            ["50", 40],
                            ["60", 55],
                            ["70", 60],
                            ["80", 65],
                            ["90", 91],
                            ["100", 110],
                ];
                    
                } else {
                    document.getElementById("radar").classList.add("waveStrength-0");
                    newPoints = [
                        ["10", 0],
                        ["20", 0],
                        ["30", 0],
                        ["40", 0],
                        ["50", 0],
                        ["60", 0],
                        ["70", 0],
                        ["80", 0],
                        ["90", 0],
                        ["100", 0],
                 ];
                }

                instance.updatePoints(newPoints);

            }
        };

        function send_joystick(speed, angle){
            var data = {"velocidade":speed, "angulo":angle};
            data = JSON.stringify(data);
            console.log('Send joystick: ', data);
            connection.send(data);
          
          /*  if(speed < 10){
                newPoints = [
                        ["10", 60],
                        ["20", 1],
                        ["30", 1],
                        ["40", 1],
                        ["50", 1],
                        ["60", 1],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
            } else if(speed >= 10 && speed < 30){
                newPoints = [
                        ["10", 40],
                        ["20", 45],
                        ["30", 50],
                        ["40", 1],
                        ["50", 1],
                        ["60", 1],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
            } else if(speed >= 30 && speed <= 60){
                newPoints = [
                        ["10", 10],
                        ["20", 10],
                        ["30", 10],
                        ["40", 30],
                        ["50", 35],
                        ["60", 40],
                        ["70", 1],
                        ["80", 1],
                        ["90", 1],
                        ["100", 1],
                ];
            } else {
                newPoints = [
                        ["10", 10],
                        ["20", 20],
                        ["30", 30],
                        ["40", 35],
                        ["50", 40],
                        ["60", 55],
                        ["70", 60],
                        ["80", 65],
                        ["90", 91],
                        ["100", 110],
              ];
            }

            instance.updatePoints(newPoints);
        }*/
    }
    </script>

    <script>
        var canvas_joystick, ctx_joystick;
        var ctx_button;
        let instance = null;

        let config = {
                    title: "",
                    x: {
                        values: ["10", "20", "30", "40","50", "60", "70","80","90","100"],
                        label: "Segundos"
                    },
                    y: {
                        values: [10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
                        label: "Distancia"
                    },
                    points: [
                        ["10", 1],
                        ["20", 2],
                        ["30", 4],
                        ["40", 6],
                        ["50", 8],
                        ["60", 9],
                        ["70", 10],
                        ["80", 9],
                        ["90", 10],
                        ["100", 9],
                        ["100", 10],

                    ],
                    options: {
                        animated: true,
                        colors: {
                            graph: "#ffffff",
                            points: "#004637",
                            lines: "#814A84",
                            axes:"#ECE5E5",
                            text: "#ECE5E5",
                        }
                    }
            }
        // setup the controls for the page
        window.addEventListener('load', () => {
            instance = new LineChart(config);

            canvas_joystick = document.getElementById('canvas_joystick');
            ctx_joystick = canvas_joystick.getContext('2d');
            
            resize();

            canvas_joystick.addEventListener('mousedown', startDrawing);
            canvas_joystick.addEventListener('mouseup', stopDrawing);
            canvas_joystick.addEventListener('mousemove', Draw);
            canvas_joystick.addEventListener('touchstart', startDrawing);
            canvas_joystick.addEventListener('touchend', stopDrawing);
            canvas_joystick.addEventListener('touchcancel', stopDrawing);
            canvas_joystick.addEventListener('touchmove', Draw);
            window.addEventListener('resize', resize);
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
            document.getElementById("button").innerText = 0;
        });

        var width, height, radius, button_size;
        let origin_joystick = { x: 0, y: 0};
        let origin_button = { x: 0, y: 0};
        const width_to_radius_ratio = 0.04;
        const width_to_size_ratio = 0.15;
        const radius_factor = 7;
            
        function resize() {
            if(window.innerWidth > window.innerHeight){
                width = window.innerHeight; // half the window for two canvases
            } else {
                width = window.innerWidth;
            }
            radius = width_to_radius_ratio * width;
            button_size = width_to_size_ratio * width;
            height = radius * radius_factor * 2 + 100; // use the diameter

            // configure and draw the joystick canvas
            ctx_joystick.canvas.width = width;
            ctx_joystick.canvas.height = height;
            origin_joystick.x = width / 2;
            origin_joystick.y = height / 2;
            joystick(origin_joystick.x, origin_joystick.y);
            
        }

        // Draw the background/outer circle of the joystick
        	function joystick_background() {
            // clear the canvas
            ctx_joystick.clearRect(0, 0, canvas_joystick.width, canvas_joystick.height);
            // draw the background circle
            ctx_joystick.beginPath();
            ctx_joystick.arc(origin_joystick.x, origin_joystick.y, radius * radius_factor, 0, Math.PI * 2, true);
            ctx_joystick.fillStyle = '#ECE5E5';
            ctx_joystick.fill();

            //seta esquerda
            ctx_joystick.beginPath();
            ctx_joystick.moveTo(origin_joystick.x - (radius * radius_factor) - 50 , origin_joystick.y);
            ctx_joystick.lineTo(origin_joystick.x - (radius * radius_factor) - 25, origin_joystick.y+25);
            ctx_joystick.lineTo(origin_joystick.x - (radius * radius_factor) - 25, origin_joystick.y-25);
            ctx_joystick.fill();
            
            //seta superior
            ctx_joystick.beginPath();
            ctx_joystick.moveTo(origin_joystick.x, origin_joystick.y - (radius * radius_factor) - 50);
            ctx_joystick.lineTo(origin_joystick.x+25, origin_joystick.y - (radius * radius_factor) - 25);
            ctx_joystick.lineTo(origin_joystick.x-25, origin_joystick.y - (radius * radius_factor) - 25);
            ctx_joystick.fill();
            
            //seta direita
            ctx_joystick.beginPath();
            ctx_joystick.moveTo(origin_joystick.x + (radius * radius_factor) + 50 , origin_joystick.y);
            ctx_joystick.lineTo(origin_joystick.x + (radius * radius_factor) + 25, origin_joystick.y+25);
            ctx_joystick.lineTo(origin_joystick.x + (radius * radius_factor) + 25, origin_joystick.y-25);
            ctx_joystick.fill();
            
            //seta inferior
            ctx_joystick.beginPath();
            ctx_joystick.moveTo(origin_joystick.x, origin_joystick.y + (radius * radius_factor) + 50);
            ctx_joystick.lineTo(origin_joystick.x+25, origin_joystick.y + (radius * radius_factor) + 25);
            ctx_joystick.lineTo(origin_joystick.x-25, origin_joystick.y + (radius * radius_factor) + 25);
            ctx_joystick.fill();
        }

        // Draw the main circle of the joystick
        function joystick(x, y) {
            // draw the background
            joystick_background();
            // draw the joystick circle
            ctx_joystick.beginPath();
            ctx_joystick.arc(x, y, radius*3, 0, Math.PI * 2, true);
            ctx_joystick.fillStyle = 'lightgray';
            ctx_joystick.fill();
            ctx_joystick.strokeStyle = 'lightgray';
            ctx_joystick.lineWidth = 2;
            ctx_joystick.stroke();
        }

        let coord = { x: 0, y: 0 };
        let paint = false;
        var movimento = 0;

        // Get the position of the mouse/touch press (joystick canvas)
        function getPosition_joystick(event) {
            var mouse_x = event.clientX || event.touches[0].clientX || event.touches[1].clientX;
            var mouse_y = event.clientY || event.touches[0].clientY || event.touches[1].clientY;
            coord.x = mouse_x - canvas_joystick.offsetLeft;
            coord.y = mouse_y - canvas_joystick.offsetTop;
        }

        // Check if the mouse/touch was pressed inside the background/outer circle of the joystick
        function in_circle() {
            var current_radius = Math.sqrt(Math.pow(coord.x - origin_joystick.x, 2) + Math.pow(coord.y - origin_joystick.y, 2));
            if ((radius * radius_factor) >= current_radius) { // consider the outer circle
                console.log("INSIDE circle");
                return true;
            } else {
                console.log("OUTSIDE circle");
                return false;
            }
        }

        // Handler: on press for the joystick canvas
        function startDrawing(event) {
            paint = true;
            getPosition_joystick(event);
            if (in_circle()) {
                // draw the new graphics
                joystick(coord.x, coord.y);
                Draw(event);
            }
        }
        // Handler: on release for the joystick canvas
        function stopDrawing() {
            paint = false; // reset

            // update to the default graphics
            joystick(origin_joystick.x, origin_joystick.y);
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
            // update the WebSocket client
            if (movimento == 1) {
                send_joystick(0, 0);
                movimento = 0;
            }
        }

        // Semi-handler: update the drawing of the joystick canvas
        function Draw(event) {
            if (paint) {
                // update the position
                getPosition_joystick(event);
                var angle_in_degrees, x, y, speed;
                // calculate the angle
                var angle = Math.atan2((coord.y - origin_joystick.y), (coord.x - origin_joystick.x));
                if (in_circle()) {
                    x = coord.x - radius / 2; // correction to center on the tip of the mouse, by why? (Thought for another time.)
                    y = coord.y - radius / 2; // correction to center on the tip of the mouse, by why? (Thought for another time.)
                } else {
                    x = radius * radius_factor * Math.cos(angle) + origin_joystick.x; // consider the outer circle
                    y = radius * radius_factor * Math.sin(angle) + origin_joystick.y; // consider the outer circle
                }

                // calculate the speed (radial coordinate) in percentage [0;100]
                var speed = Math.round(100 * Math.sqrt(Math.pow(x - origin_joystick.x, 2) + Math.pow(y - origin_joystick.y, 2)) / (radius * radius_factor)); // consider the outer circle
                if (speed > 100){
                    speed = 100; // limit
                }

                // convert the angle to degrees [0;360]
                if (Math.sign(angle) == - 1) {
                    angle_in_degrees = Math.round( - angle * 180 / Math.PI);
                }
                else {
                    angle_in_degrees = Math.round(360 - angle * 180 / Math.PI);
                }

                // update the elements
                joystick(x, y);
                document.getElementById("speed").innerText = speed;
                document.getElementById("angle").innerText = angle_in_degrees;
                // send the data
                send_joystick(speed, angle_in_degrees, instance);
                movimento = 1;
            }
        }

    </script>

</body>
</html>
)rawliteral";

// --------------------------------------------------
// Prototipos

void configurar_servidor_web(void);
void handleWebSocketMessage(void *, uint8_t *, size_t);
//void handleButton(void *, uint8_t *, size_t);

int16_t ler_distancia(void);
void onEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType,
             void *, uint8_t *, size_t);

// --------------------------------------------------
// --------------------------------------------------

void setup(){
  // configura a comunicacao serial
  Serial.begin(115200);
  Serial.println("RoboCore - Vespa Joystick");
  Serial.println("\t(v1.0 - 25/10/21)\n");


  // configura o LED
  pinMode(PINO_LED, OUTPUT);
  digitalWrite(PINO_LED, LOW);

  // configura o sensor ultrassonico
  pinMode(PINO_HCSR04_ECHO, INPUT);
  pinMode(PINO_HCSR04_TRIGGER, OUTPUT);
  digitalWrite(PINO_HCSR04_TRIGGER, LOW);

  // configura o ponto de acesso (Access Point)
  Serial.print("Configurando a rede Wi-Fi... ");
  const char *mac = WiFi.macAddress().c_str(); // obtem o MAC
  char ssid[] = "Vespa-xxxxx"; // mascara do SSID (ate 63 caracteres)
  char *senha = "thebobsuper"; // senha padrao da rede (no minimo 8 caracteres)
  // atualiza o SSID em funcao do MAC
  for(uint8_t i=6 ; i < 11 ; i++){
    ssid[i] = mac[i+6];
  }
  if(!WiFi.softAP(ssid, senha)){
    Serial.println("ERRO");
    // trava a execucao
    while(1){
      digitalWrite(PINO_LED, HIGH);
      delay(100);
      digitalWrite(PINO_LED, LOW);
      delay(100);
    }
  }
  
  Serial.println("OK");
  Serial.printf("A rede \"%s\" foi gerada\n", ssid);
  Serial.print("IP de acesso: ");
  Serial.println(WiFi.softAPIP());

  // configura e iniciar o servidor web
  configurar_servidor_web();
  server.begin();
  Serial.println("Servidor iniciado\n");
  //pinMode(SENSOR_LINHA_ESQUERDO, INPUT);
  //pinMode(SENSOR_LINHA_DIREITO, INPUT);
}

// --------------------------------------------------
void anticolisao() {
  
  delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
  //armazena a distancia lida pelo sensor a variavel distancia
  distancia_2 = (sensor_ultrassonico.ping()) / 58;

  //verifica se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
  if (distancia_2 <= DISTANCIA_OBSTACULO && distancia_2 != 0) { //se for verdadeiro

    delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
    distancia_2 = (sensor_ultrassonico.ping()) / 58; //atualiza a leitura do sensor

    //confirma se a distancia lida pelo sensor e menor ou igual ao valor configurado na variavel "DISTANCIA_OBSTACULO"
    if (distancia_2 <= DISTANCIA_OBSTACULO && distancia_2 != 0) {

      motores.stop(); //para os motores do robo
      motores.backward(VELOCIDADE); //recua o robo girando os motores para tras
      delay(ESPERA_MOVIMENTO/2); //mantem o movimento do robo
      motores.stop(); //para os motores do robo

      //verifica se o tempo de execucao do codigo e par ou impar
      if (millis() % 2 == 0) {  //se for par

        motores.turn(VELOCIDADE, -VELOCIDADE); //gira o robo para a direita
        delay(ESPERA_MOVIMENTO); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        distancia_2 = (sensor_ultrassonico.ping()) / 58; //atualiza a leitura do sensor

        //enquanto a leitura do sensor for menor que o valor da variavel "DISTANCIA_OBSTACULO"
        while (distancia_2 <= DISTANCIA_OBSTACULO && distancia_2 != 0) {

          motores.turn(VELOCIDADE, -VELOCIDADE); //mantem o robo girando para a direita
          delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
          distancia_2 = (sensor_ultrassonico.ping()) / 58; //atualiza a leitura do sensor

        }

        //movimentacao de contorno do obstaculo
        motores.forward(VELOCIDADE); //movimenta o robô para frente
        delay(ESPERA_MOVIMENTO*2); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        motores.turn(-VELOCIDADE, VELOCIDADE); //gira o robo para a esquerda
        delay(ESPERA_MOVIMENTO); //mantem o movimento do robo
        motores.forward(VELOCIDADE); //movimenta o robo para frente
        delay(ESPERA_MOVIMENTO*2); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        motores.turn(-VELOCIDADE, VELOCIDADE); //gira o robo para a esquerda
        delay(ESPERA_MOVIMENTO*1.5); //mantem o movimento do robo
        motores.forward(VELOCIDADE); //movimenta o robo para frente

      } else { //se o tempo de execucao do codigo for impar

        motores.turn(-VELOCIDADE, VELOCIDADE); //gira o robo para a esquerda
        delay(ESPERA_MOVIMENTO); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        distancia_2 = (sensor_ultrassonico.ping()) / 58; //atualiza a leitura do sensor

        //enquanto a leitura do sensor for menor que o valor da variavel "DISTANCIA_OBSTACULO"
        while (distancia_2 <= DISTANCIA_OBSTACULO && distancia_2 != 0) {

          motores.turn(-VELOCIDADE, VELOCIDADE); //mantem o robo girando para a direita
          delay(ESPERA); //aguarda o tempo de espera para leitura do sensor
          distancia_2 = (sensor_ultrassonico.ping()) / 58; //atualiza a leitura do sensor

        }

        //movimentacao de contorno do obstaculo
        motores.forward(VELOCIDADE); //movimenta o robô para frente
        delay(ESPERA_MOVIMENTO*2); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        motores.turn(VELOCIDADE, -VELOCIDADE); //gira o robo para a direita
        delay(ESPERA_MOVIMENTO); //mantem o movimento do robo
        motores.forward(VELOCIDADE); //movimenta o robo para frente
        delay(ESPERA_MOVIMENTO*2); //mantem o movimento do robo
        motores.stop(); //para os motores do robo
        motores.turn(VELOCIDADE, -VELOCIDADE); //gira o robo para a direita
        delay(ESPERA_MOVIMENTO*1.5); //mantem o movimento do robo
        motores.forward(VELOCIDADE); //movimenta o robo para frente

      }
    }
  }
  
}

void seguidor_linha() {
  
  //realiza a leitura dos sensores
  leitura_esquerdo = analogRead(SENSOR_LINHA_ESQUERDO);
  leitura_direito = analogRead(SENSOR_LINHA_DIREITO);

  //verifica se ambas as leituras dos sensores sao maiores que o valor de leitura de corte
  //ou seja se os dois sensores estao sobre a linha da pista
  if ((leitura_esquerdo > LEITURA_LINHA) && (leitura_direito > LEITURA_LINHA)) { //se for verdadeiro
    //movimenta o robo para frente
    motores.forward(VELOCIDADE);
    contador_parada = 0; //zera o contador de parada
  }

  //verifica se ambas as leituras dos sensores sao menores que o valor de leitura de corte
  //ou seja se os dois sensores estao fora da linha da pista
  else if ((leitura_esquerdo < LEITURA_LINHA) && (leitura_direito < LEITURA_LINHA)) { //se for verdadeiro
    contador_parada++; //incrementa o contador de parada
  }

  //verifica se apenas a leitura do sensor da direita e menor que o valor de leitura de corte
  //ou seja se apenas o sensor da direta esta sobre a linha da pista
  else if (leitura_direito > LEITURA_LINHA) { //se for verdadeiro
    //gira o robo para a esquerda ajustando a velocidade dos motores
    motores.turn(VELOCIDADE + VELOCIDADE_SOMA, VELOCIDADE - VELOCIDADE_SUBTRACAO);
    contador_parada = 0; //zera o contador de parada
  }


  //verifica se apenas a leitura do sensor da esquerda e menor que o valor de leitura de corte
  //ou seja se apenas o sensor da esquerda esta sobre a linha da pista
  else if (leitura_esquerdo > LEITURA_LINHA) {
    //gira o robo para a direita ajustando a velocidade dos motores
    motores.turn(VELOCIDADE - VELOCIDADE_SUBTRACAO, VELOCIDADE + VELOCIDADE_SOMA);
    contador_parada = 0; //zera o contador de parada
  }

  //verifica se o contador de parada e maior ou igual que o valor de contagem maxima
  //ou seja se o robo ficou muito tempo fora da pista
  if (contador_parada >= CONTAGEM_MAXIMA) { //se for verdadeiro
    motores.stop(); //para o robo
  }
  
}


void loop() {
 //   if(ws.count() > 0){
   //     if(global_ativo){
        //executa a funcao de controle de seguidor de linha
   //     seguidor_linha();
   //     //executa a funcao de controle de anticolisao
  //      anticolisao();
  //   }else{
  //      motores.stop();
  //   }
 //   }
  if(millis() > timeout_vbat){
    // atualiza se houver clientes conectados
    if(ws.count() > 0){
      // le a tensao da bateria
      uint32_t tensao = vbat.readVoltage();
      
      // cria a mensagem
      const int json_tamanho = JSON_OBJECT_SIZE(1); // objeto JSON com um membro
      StaticJsonDocument<json_tamanho> json;
      json[ALIAS_VBAT] = tensao;
      size_t mensagem_comprimento = measureJson(json);
      char mensagem[mensagem_comprimento + 1];
      serializeJson(json, mensagem, (mensagem_comprimento+1));
      mensagem[mensagem_comprimento] = 0; // EOS (mostly for debugging)
  
      // send the message
      ws.textAll(mensagem, mensagem_comprimento);
      Serial.printf("Tensao atualizada: %u mV\n", tensao);
    }
    
    timeout_vbat = millis() + TEMPO_ATUALIZACAO_VBAT; // atualiza
  }
  
  // le a distancia lida pelo sensor e envia para o cliente
  if(millis() > timeout_distancia){
    // atualiza se houver clientes conectados
    if(ws.count() > 0){
      // le a tensao da bateria
      float fator = 0.2; // entre 0.0 e 1.0
      float leitura = ler_distancia();
      distancia = leitura * fator + (1.0 - fator) * (float)distancia;
      
      // cria a mensagem
      const int json_tamanho = JSON_OBJECT_SIZE(1); // objeto JSON com um membro
      StaticJsonDocument<json_tamanho> json;
      json[ALIAS_DISTANCIA] = distancia;
      size_t mensagem_comprimento = measureJson(json);
      char mensagem[mensagem_comprimento + 1];
      serializeJson(json, mensagem, (mensagem_comprimento+1));
      mensagem[mensagem_comprimento] = 0; // EOS (mostly for debugging)
  
      // send the message
      ws.textAll(mensagem, mensagem_comprimento);
      Serial.printf("Distancia atualizada: %u cm\n", distancia);
    }
    
    
    timeout_distancia = millis() + TEMPO_ATUALIZACAO_DISTANCIA; // atualiza
  }  
}

// --------------------------------------------------
// --------------------------------------------------

// Configurar o servidor web
void configurar_servidor_web(void) {
  ws.onEvent(onEvent); // define o manipulador do evento do WebSocket
  server.addHandler(&ws); // define o manipulador do WebSocket no servidor
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ // define a resposta da pagina padrao
    request->send_P(200, "text/html", index_html);
  });
}

// --------------------------------------------------

// Manipulador para mensagens WebSocket
//  @param (arg) : xxx [void *]
//         (data) : xxx [uint8_t *]
//         (length) : xxx [size_t]


void handleButton(void *arg, uint8_t *data, size_t length) {
    data[length] = 0;

    const int json_tamanho = JSON_OBJECT_SIZE(1); // objeto JSON com dois membros
    StaticJsonDocument<json_tamanho> json;
    DeserializationError erro = deserializeJson(json, data, length);
    global_ativo = json[ALIAS_ACTIVE]; 
    
}
 

void handleWebSocketMessage(void *arg, uint8_t *data, size_t length) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == length && info->opcode == WS_TEXT) {
    data[length] = 0;
//    Serial.printf("Incoming WS data: \"%s\"\n", (char*)data); // debug

    // verifica se eh para controlar os motores
    if(strstr(reinterpret_cast<char*>(data), ALIAS_VELOCIDADE) != nullptr){
      // cria um documento JSON
      const int json_tamanho = JSON_OBJECT_SIZE(3); // objeto JSON com dois membros
      StaticJsonDocument<json_tamanho> json;
      DeserializationError erro = deserializeJson(json, data, length);
      
      // extrai os valores do JSON
      int16_t angulo = json[ALIAS_ANGULO]; // [0;360]
      int16_t velocidade = json[ALIAS_VELOCIDADE]; // [0;100]
      
      Serial.print("Robo ativo: ");

      // debug
      Serial.print("Velocidade: ");
      Serial.print(velocidade);
      Serial.print(" | Angulo: ");
      Serial.println(angulo);

      // atualiza os motores

      //curva frente para a esquerda
      if((angulo >= 90) && (angulo <= 180)){
        motores.turn(velocidade * (135 - angulo) / 45 , velocidade);

      //curva frente para a direita  
      } else if((angulo >= 0) && (angulo < 90)){
        motores.turn(velocidade, velocidade * (angulo - 45) / 45);

      //curva tras esquerda   
      } else if((angulo > 180) && (angulo <= 270)){
        motores.turn(-1 * velocidade, -1 * velocidade * (angulo - 225) / 45);

      //curva tras direita   
      } else if(angulo > 270){
        motores.turn(-1 * velocidade * (315 - angulo) / 45, -1 * velocidade);

      } else {
        motores.stop();
      }

    } else {
      Serial.printf("Recebidos dados invalidos (%s)\n", data);
    }
  }
}

// --------------------------------------------------

// Ler a distancia com o sensor ultrassonico (HC-SR04)
int16_t ler_distancia(void){
  // realiza um pulso de 10 microsegundos no trigger do sensor
  digitalWrite(PINO_HCSR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_HCSR04_TRIGGER, LOW);

  // mede o pulso em microsegundos retornado para o echo do sensor
  // e converte o tempo para distancia divindo por 58
  return pulseIn(PINO_HCSR04_ECHO, HIGH) / 58;
  
}

// --------------------------------------------------

// Manipulador dos eventos do WebSocket
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT: {
      digitalWrite(PINO_LED, HIGH); // acende o LED
      // permitir apenas um cliente conectado
      if(ws.count() == 1){ // o primeiro cliente ja eh considerado como conectado
        Serial.printf("Cliente WebSocket #%u conectado de %s\n", client->id(), client->remoteIP().toString().c_str());
      } else {
        Serial.printf("Cliente WebSocket #%u de %s foi rejeitado\n", client->id(), client->remoteIP().toString().c_str());
        ws.close(client->id());
      }
      break;
    }
    case WS_EVT_DISCONNECT: {
      if(ws.count() == 0){
        digitalWrite(PINO_LED, LOW); // apaga o LED
      }
      Serial.printf("Cliente WebSocket #%u desconectado\n", client->id());
      break;
    }
    case WS_EVT_DATA: {
      handleWebSocketMessage(arg, data, length);
     // handleButton(arg, data, length);
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}





// --------------------------------------------------