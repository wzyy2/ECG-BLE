.pragma library

var default_chartOptions = {
    pointDot : false,
    scaleShowGridLines : false,
    animation: false,
    scaleSteps : 10,
    scaleStepWidth : 50,
    scaleStartValue : 500,
    //scaleOverride : true,
    animationSteps : 0,
    bezierCurve : false,

  };


var default_chartData = {
      labels: [],
      datasets: [
          {
              label: "My Second dataset",
              fillColor: "rgba(151,187,205,0.2)",
              strokeColor: "rgba(151,187,205,1)",
              pointColor: "rgba(151,187,205,1)",
              pointStrokeColor: "#fff",
              pointHighlightFill: "#fff",
              pointHighlightStroke: "rgba(151,187,205,1)",
              data: []
          },
      ]
  };


var newLabels=[];
for(var i=0;i<500;i++){
    newLabels.push('');
}
default_chartData["labels"]=newLabels;

