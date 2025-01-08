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

const polygon1 = new Polygon([0, 0, 160], [0, 284, 284]);
const polygon2 = new Polygon([0, 160, 236, 236], [0, 284, 130, 203]);
const polygon3 = new Polygon([0, 236, 236], [0, 0, 130]);

const polygon4 = new Polygon([0, 236, 236], [0, 0, -130]); 
const polygon5 = new Polygon([0, 236, 120, 236], [0, -130, -203, -203]);
const polygon6 = new Polygon([0, 120, 0], [0, -203, -203]);

const polygon7 = new Polygon([0, 0, -120], [0, -203, -203]);
const polygon8 = new Polygon([0, -120, -175, -232], [0, -203, -102, -203]);
const polygon9 = new Polygon([0, -232, -236], [0, -130, 0]);

const polygon10 = new Polygon([0, -236, -232], [0, 0, 130]);
const polygon11 = new Polygon([0, -232, -116, -232], [0, 130, 203, 203]);
const polygon12 = new Polygon([0, -116, 0], [0, 200, 203]);

let inPicker = false;
locationImg.onmouseenter = () => inPicker = true;
locationImg.onmouseleave = () => {
    if(!areaSelected) {
        inPicker = false;
        locationImg.src = "locationSelectorImages/locationSelectorNone.png";
    }
}

document.addEventListener("mousemove", (event) => {
    if(menu == "reef" && inPicker && !areaSelected) {
        let imgPosition = locationImg.getBoundingClientRect();
        let imageCenterX = imgPosition.left + (imgPosition.right-imgPosition.left) / 2;
        let imageCenterY = imgPosition.top + (imgPosition.bottom-imgPosition.top) / 2;
        
        let x = event.clientX-imageCenterX;
        let y = imageCenterY-event.clientY;

        if(x > 0 && y > 0) /*helps performance*/ {
            if(pointInPolygon(polygon1, [x, y])) {
                reefArea = 1;
            } else if (pointInPolygon(polygon2, [x, y])) {
                reefArea = 2;
            } else if (pointInPolygon(polygon3, [x, y])) {
                reefArea = 3;
            }
        } else if (x > 0 && y < 0) {
            if(pointInPolygon(polygon4, [x, y])) {
                reefArea = 4;
            } else if (pointInPolygon(polygon5, [x, y])) {
                reefArea = 5;
            } else if (pointInPolygon(polygon6, [x, y])) {
                reefArea = 6;
            }
        } else if (x < 0 && y < 0) {
            if(pointInPolygon(polygon7, [x, y])) {
                reefArea = 7;
            } else if (pointInPolygon(polygon8, [x, y])) {
                reefArea = 8;
            } else if (pointInPolygon(polygon9, [x, y])) {
                reefArea = 9;
            }
        } else if (x < 0 && y > 0) {
            if(pointInPolygon(polygon10, [x, y])) {
                reefArea = 10;
            } else if (pointInPolygon(polygon11, [x, y])) {
                reefArea = 11;
            } else if (pointInPolygon(polygon12, [x, y])) {
                reefArea = 12;
            }
        }
        locationImg.src = `locationSelectorImages/locationSelector${reefArea}.png`;
    }
});

let areaText = document.getElementById("areaText");

locationImg.onclick = () => {
    areaSelected = !areaSelected;
    if(areaSelected) {
        areaText.innerText = `Reef Area: ${reefArea}`;
    }
    if(coralSelected && areaSelected) {
        confirmReefButton.innerText = `Score at Level ${coralLevel} and Area ${reefArea}`;
    }
    if(!areaSelected) {
        confirmReefButton.innerText = `Choose Robot Alignment`;
        areaText.innerText = `Reef Area: None`;
    }
}