const reefContainer = document.querySelector(".reefContainer");
const processorContainer = document.querySelector(".processorContainer");
const confirmReefContainer = document.querySelector(".confirmChoice");
const confirmReefButton = document.getElementById("confirmText");
const locationContainer = document.querySelector(".locationContainer");
const screenChangeContainer = document.querySelector(".screenChangeContainer");
const choiceContainer = document.querySelector(".choiceContainer");

let menu;

let coralLevel = -1;
let reefArea = -1;
let coralSelected = false;
let areaSelected = false;

const changeMenu = (menuVal, screenmsg) => {
    menu = menuVal;
    reefContainer.style.display = "none";
    locationContainer.style.display = "none";
    screenChangeContainer.style.display = "none";
    processorContainer.style.display = "none";
    choiceContainer.style.display = "none";
    confirmReefContainer.style.display = "none";
    if (menu == "processor") {
        processorContainer.style.display = "";
    } else if (menu == "reef"){
        reefContainer.style.display = "";
        locationContainer.style.display = "";
        confirmReefContainer.style.display = "";
        confirmReefButton.innerText = "Choose Robot Alignment";
    } else if (menu == "screen change") {
        screenChangeContainer.style.display = "";
        document.getElementById("msgDisplayer").innerText = screenmsg;
    } else if (menu == "choice") {
        choiceContainer.style.display = "";
    }
}

changeMenu("choice");

const reefChoice = document.getElementById("reefChoice");
const processorChoice = document.getElementById("processorChoice");

reefChoice.onclick = () => {
    changeMenu("reef");
}

processorChoice.onclick = () => {
    changeMenu("processor");
}

let reefScoreButtonClickable = true;
confirmReefButton.onclick = async () => {
    if(reefScoreButtonClickable) {
        reefScoreButtonClickable = false;
        confirmReefButton.innerText = "Scoring...";
        await scoreReef(reefArea, coralLevel);
        confirmReefButton.innerText = "Choose Robot Alignment";
        changeMenu("choice");
        reefScoreButtonClickable = true;

        coralSelected = false;
        reefSelected = false;
        coralLevel = -1;
        document.getElementById("coral").src = `coralLevelImages/coralNone.png`;
    }
}