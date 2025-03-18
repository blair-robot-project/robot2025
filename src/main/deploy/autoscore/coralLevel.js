const levelOne = document.getElementById("L1");
const levelTwo = document.getElementById("L2");
const levelThree = document.getElementById("L3");
const levelFour = document.getElementById("L4");

const levelList = [levelOne, levelTwo, levelThree, levelFour];

const coralImage = document.getElementById("coral");
coralLevel = -1;
let prevCoralLevel = -1;

const setCoralImg = (level) => {
    coralImage.src = `coralLevelImages/coral${level}.png`;
}

["mousemove", "touchmove"].forEach(_ => coralImage.addEventListener(_, (event) => {
    //this is radius not actual width
    const coralImgPos = coralImage.getBoundingClientRect();
    const coralHeight = (coralImgPos.bottom-coralImgPos.top) / 2;
    const coralY = coralImgPos.top + coralHeight;
    let mouseY = event.clientY;
    let ydiff = coralY - mouseY;
    if(_ == "touchmove") {
        event.preventDefault();
        mouseY = event.changedTouches[0].clientY;
        ydiff = coralY - mouseY;
    }
    if(!coralSelected) {
        if(ydiff > coralHeight * 0.4) {
            setCoralImg("L4");
            prevCoralLevel = 4;
        } else if (ydiff > coralHeight * 0.1) {
            setCoralImg("L3");
            prevCoralLevel = 3;
        } else if (ydiff > -coralHeight * 0.4) {
            setCoralImg("L2");
            prevCoralLevel = 2;
        } else {
            setCoralImg("L1");
            prevCoralLevel = 1;
        }
    }   
}));


["mouseleave", "touchend"].forEach(_ => coralImage.addEventListener(_, () => {
    if(coralSelected) {
        if(coralLevel == 1) {
            setCoralImg("L1");
        } else if (coralLevel == 2) {
            setCoralImg("L2");
        } else if (coralLevel == 3) {
            setCoralImg("L3");
        } else if (coralLevel == 4) {
            setCoralImg("L4");
        }
    } else {
        setCoralImg("None");
    }
}));

coralImage.onclick = () => {
    coralSelected = !coralSelected;
    if(coralSelected) {
        coralLevel = prevCoralLevel;
        coralImage.src = `coralLevelImages/coralL${coralLevel}.png`;
        if(areaSelected) {
            confirmReefButton.style.backgroundColor = "rgb(4, 189, 36)";
            confirmReefButton.innerText = `Score at Level ${coralLevel} and Area ${numberToLetter[11-(reefArea+4)%12]}`;
        } else {
            confirmReefButton.innerText = `Coral Level: L${coralLevel}`;
        }
    } else {
        setCoralImg("None");
        coralLevel = -1;
        confirmReefButton.style.backgroundColor = "#DD0000";
        if(areaSelected) {
            confirmReefButton.innerText = `Reef Area: ${numberToLetter[11-(reefArea+4)%12]}`;
        } else {
            confirmReefButton.innerText = `Choose Robot Alignment`;
        }
    }
    
}