const locationImg = document.getElementById("locationSelect");
const pointInPolygon = function (polygon, point) {
    //A point is in a polygon if a line from the point to infinity crosses the polygon an odd number of times
    let odd = false;
    //For each edge (In this case for each point of the polygon and the previous one)
    for (let i = 0, j = polygon.xp.length - 1; i < polygon.xp.length; i++) {
        //If a line from the point into infinity crosses this edge
        if (((polygon.yp[i] > point[1]) !== (polygon.yp[j] > point[1])) // One point needs to be above, one below our y coordinate
            // ...and the edge doesn't cross our Y corrdinate before our x coordinate (but between our x coordinate and infinity)
            && (point[0] < ((polygon.xp[j] - polygon.xp[i]) * (point[1] - polygon.yp[i]) / (polygon.yp[j] - polygon.yp[i]) + polygon.xp[i]))) {
            // Invert odd
            odd = !odd;
        }
        j = i;

    }
    //If the number of crossings was odd, the point is in the polygon
    return odd;
};

class Polygon {
    constructor(xp, yp) {
        this.xp = xp;
        this.yp = yp;
    }
}

let imageCenterX = 682; //440 with console, 682 without console
let imageCenterY = 372;

const polygon1 = new Polygon([0, 0, 160], [0, 284, 284]);
const polygon2 = new Polygon([0,160, 171], [0, 284, 104]);
const polygon3 = new Polygon([0, 171, 232], [0, 104, 0]);

const polygon4 = new Polygon([0, 232, -176], [0, 0, -100]); //-176, 100
const polygon5 = new Polygon([0, -176, 120], [0, 100, -200]); //120, -200
const polygon6 = new Polygon([0, 120, 0], [0, -200, -200]); //0, -200

const polygon7 = new Polygon([0, 0, -116], [0, -200, -200]); //-116, -200
const polygon8 = new Polygon([0, -116, -175], [0, -200, -102]);
const polygon9 = new Polygon([0, -176, -232], [0, -102, 0]);  //-232, 0

const polygon10 = new Polygon([0, -232, -172], [0, 0, 100]); //-172, 100
const polygon11 = new Polygon([0, -172, -116], [0, 100, 200]); //-116, 200
const polygon12 = new Polygon([0, -116, 0], [0, 200, 200]); //0, 200

let inPicker = false;
locationImg.onmouseenter = () => inPicker = true;
locationImg.onmouseleave = () => {
    inPicker = false;
    locationImg.src = "locationSelectorNone.png";
}

let locationArea = 1;


document.addEventListener("mousemove", (event) => {
    if(menu == "location pick" && inPicker) {
        let x = event.screenX-imageCenterX;
        let y = imageCenterY-event.screenY;
        if(x > 0 && y > 0) /*helps performance*/ {
            if(pointInPolygon(polygon1, [x, y])) {
                locationArea = 1;
            } else if (pointInPolygon(polygon2, [x, y])) {
                locationArea = 2;
            } else if (pointInPolygon(polygon3, [x, y])) {
                locationArea = 3;
            }
        } else if (x > 0 && y < 0) {
            if(pointInPolygon(polygon4, [x, y])) {
                locationArea = 4;
            } else if (pointInPolygon(polygon5, [x, y])) {
                locationArea = 5;
            } else if (pointInPolygon(polygon6, [x, y])) {
                locationArea = 6;
            }
        } else if (x < 0 && y < 0) {
            if(pointInPolygon(polygon7, [x, y])) {
                locationArea = 7;
            } else if (pointInPolygon(polygon8, [x, y])) {
                locationArea = 8;
            } else if (pointInPolygon(polygon9, [x, y])) {
                locationArea = 9;
            }
        } else if (x < 0 && y > 0) {
            if(pointInPolygon(polygon10, [x, y])) {
                locationArea = 10;
            } else if (pointInPolygon(polygon11, [x, y])) {
                locationArea = 11;
            } else if (pointInPolygon(polygon12, [x, y])) {
                locationArea = 12;
            }
        }
        locationImg.src = `locationSelector${locationArea}.png`;
    }
});

const timeoutList = [];

locationImg.onclick = async () => {
    changeMenu("screen change", `Moving to Reef Location ${locationArea}`);
    moveToReefLocation(locationArea);
    timeoutList.map((timeout) => clearTimeout(timeout));
    //this is just a demo wait time to simulate
    //the robot moving to the right location
    await new Promise((resolve) => {
        const waitTimeout = setTimeout(resolve, 3000);
        timeoutList.push(waitTimeout);
    });
    changeMenu("reef");
}