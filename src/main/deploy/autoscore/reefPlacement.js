const locationImg = document.getElementById("locationSelect");
const confirmReefButton = document.getElementById("confirmReefButton");

["touchmove", "mousemove"].forEach(_ => {
    locationImg.addEventListener(_, (event) => {
        if(!areaSelected) {
            let imgPosition = locationImg.getBoundingClientRect();
            let imageCenterX = imgPosition.left + (imgPosition.right-imgPosition.left) / 2;
            let imageCenterY = imgPosition.top + (imgPosition.bottom-imgPosition.top) / 2;
            
            let x = event.clientX-imageCenterX;
            let y = imageCenterY-event.clientY;
            if(_ == "touchmove") {
                event.preventDefault();
                x = event.changedTouches[0].clientX-imageCenterX;
                y = imageCenterY-event.changedTouches[0].clientY;
            }
    
            let angleRadians = Math.atan2(x, y);
            let angleDegrees = angleRadians * (180 / Math.PI);
            if(angleRadians < 0) {
                angleDegrees = 360 + angleRadians * (180 / Math.PI);
            }
            reefArea = Math.floor((angleDegrees % 360)/30) + 1;
            if(reefArea == -1) {
                locationImg.src = `locationSelectorImages/locationSelectorNone.png`;
            } else {
                locationImg.src = `locationSelectorImages/locationSelector${reefArea}.png`;
            }
        }
    });
});

["touchend", "mouseleave"].forEach(_ => {
    locationImg.addEventListener(_, () => {
        if(!areaSelected) {
            locationImg.src = "locationSelectorImages/locationSelectorNone.png";
        }
    });
});


locationImg.onclick = () => {
    areaSelected = !areaSelected;
    if(areaSelected) {
        if(coralSelected) {
            confirmReefButton.style.backgroundColor = "rgb(4, 189, 36)";
            confirmReefButton.innerText = `Score at Level ${coralLevel} and Area ${numberToLetter[11-(reefArea+4)%12]}`;
        } else {
            confirmReefButton.innerText = `Reef Area: ${numberToLetter[11-(reefArea+4)%12]}`;
        }
    } else {
        reefArea = -1;
        if(coralSelected) {
            confirmReefButton.innerText = `Coral Level: L${coralLevel}`;
        } else {
            confirmReefButton.innerText = `Choose Robot Alignment`;
        }
        locationImg.src = "locationSelectorImages/locationSelectorNone.png";
        confirmReefButton.style.backgroundColor = "#DD0000";
    }
    
}