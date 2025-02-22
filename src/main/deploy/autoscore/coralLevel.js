const levelOne = document.getElementById("L1");
const levelTwo = document.getElementById("L2");
const levelThree = document.getElementById("L3");
const levelFour = document.getElementById("L4");

const levelList = [levelOne, levelTwo, levelThree, levelFour];

const coralText = document.getElementById("coralText");
const coralImage = document.getElementById("coral");
coralLevel = -1;
let prevCoralLevel = -1;

const setCoralImg = (level) => {
    coralImage.src = `coralLevelImages/coral${level}.png`;
}

const coralImgPos = coralImage.getBoundingClientRect();
//this is radius not actual width
const coralWidth = (coralImgPos.right-coralImgPos.left) / 2;
const coralHeight = (coralImgPos.bottom-coralImgPos.top) / 2;
const coralX = coralImgPos.left + coralWidth;
const coralY = coralImgPos.top + coralHeight;
coralImage.addEventListener("mousemove", (event) => {
    let mouseY = event.clientY;
    let ydiff = coralY - mouseY;
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
});
coralImage.addEventListener("mouseleave", () => {
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
})

coralImage.onclick = () => {
    coralSelected = !coralSelected;
    if(coralSelected) {
        coralLevel = prevCoralLevel;
        coralImage.src = `coralLevelImages/coralL${coralLevel}.png`;
        coralText.innerText = `Coral Level: ${coralLevel}`;
        if(areaSelected) {
            confirmReefButton.innerText = `Score at Level ${coralLevel} and Area ${reefArea}`;
        }
    } else {
        setCoralImg("None");
        coralLevel = -1;
        coralText.innerText = "Coral Level: None";  
    }
    
}