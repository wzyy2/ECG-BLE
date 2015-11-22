/***************************************************************************
**
** Copyright (C) 2014 BlackBerry Limited. All rights reserved.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.0
import QtQuick.Particles 2.0
import "./jbQuick/Charts"
import "chart.js" as Global


Rectangle {
    id: screenMonitor
    color: "#F0EBED"

    Button {
        id:menu
        buttonWidth: parent.width
        buttonHeight: 0.1 * parent.height
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        text: "Display Wave"
        onButtonClick: {
            chart_line.chart = false; // HACK. Force it to update.
            chart_line.repaint();
            //heartRate.disconnectService();
            //pageLoader.source="home.qml";
        }
    }

    Text {
        id: hrValue
        font.pointSize: 24; font.bold: true
        anchors.top:menu.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 25

        color: "#3870BA"
        text: heartRate.hrstr
        onTextChanged: {
            if (heartRate.hr > 0 && updatei != null && heartRate.numDevices() > 0) {
                updatei.destroy()
            }
        }
    }

    Rectangle {
        id: updatei
        width: parent.width
        height: 80
        anchors.bottom: stop.top

        color: "#F0EBED"
        border.color: "#3870BA"
        border.width: 2

        Text {
            id: logi
            text: heartRate.message
            anchors.centerIn: updatei
            color: "#3870BA"
        }
    }

    Chart {
      id: chart_line;
      anchors.centerIn: parent
      width: parent.width;
      height: 400;
      chartAnimated: false;
      chartAnimationEasing: Easing.InOutElastic;
      chartAnimationDuration: 0;
      chartType: Charts.ChartType.LINE;

        property int test_val : 0;
      property var ecg_data: heartRate.ecg_data
      onEcg_dataChanged : {

          for(var i=0; i<ecg_data.length; i += 1) {
              if(chartData.datasets[0].data.length >= 500)
                chartData.datasets[0].data.shift();
              chartData.datasets[0].data.push(ecg_data[i]);
          }


//          test_val++;
//          if(test_val == 25) {
//              test_val = 0;
//              chart = false; // HACK. Force it to update.
//              repaint();
//          }
      }

      Component.onCompleted: {

        chartOptions = Global.default_chartOptions;
        chartData = Global.default_chartData;
      }
    }

    Image {
        id: background
        width: 300
        height: 0
        anchors.centerIn: parent
        source: "blue_heart.png"
        fillMode: Image.PreserveAspectFit
        NumberAnimation on width {
            running: heartRate.hr > 0;
            duration: heartRate.hr/60*250;
            from:300; to: 350;
            loops: Animation.Infinite;
        }

//        ParticleSystem {
//            id: systwo
//            anchors.fill: parent

//            ImageParticle {
//                system: systwo
//                id: cptwo
//                source: "star.png"
//                colorVariation: 0.4
//                color: "#000000FF"
//            }

//            Emitter {
//                //burst on click
//                id: burstytwo
//                system: systwo
//                enabled: true
//                x: 160
//                y: 150
//                emitRate: heartRate.hr*100
//                maximumEmitted: 4000
//                acceleration: AngleDirection {angleVariation: 360; magnitude: 360; }
//                size: 4
//                endSize: 8
//                sizeVariation: 4
//            }


//        }

    }

    Button {
        id:stop
        buttonWidth: parent.width
        buttonHeight: 0.1*parent.height
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        text: "Stop Monitoring"
        onButtonClick: {
            //burstytwo.enabled = false;
            heartRate.disconnectService();
            pageLoader.source = "results.qml";
        }
    }



}
