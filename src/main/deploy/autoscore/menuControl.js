const reefContainer = document.querySelector(".reefContainer");
const processorContainer = document.querySelector(".processorContainer");
const confirmReefContainer = document.querySelector(".confirmChoice");
const confirmReefButton = document.getElementById("confirmText");
const locationContainer = document.querySelector(".locationContainer");
const screenChangeContainer = document.querySelector(".screenChangeContainer");
const choiceContainer = document.querySelector(".choiceContainer");
const netContainer = document.querySelector(".netContainer");
const coralIntakeContainer = document.querySelector(".coralIntakeContainer");

let menu;

let coralLevel = -1;
let reefArea = -1;
let coralSelected = false;
let areaSelected = false;

const menuChoices = {
    PROCESSOR: "processor",
    REEF: "reef",
    SCREEN: "screen",
    CHOICE: "choice",
    NET: "net",
    CORAL_INTAKE: "coral intake"
}

const changeMenu = (menuVal, screenmsg) => {
    menu = menuVal;
    reefContainer.style.display = "none";
    locationContainer.style.display = "none";
    screenChangeContainer.style.display = "none";
    processorContainer.style.display = "none";
    choiceContainer.style.display = "none";
    confirmReefContainer.style.display = "none";
    netContainer.style.display = "none";
    coralIntakeContainer.style.display = "none";
    if (menu == "processor") {
        processorContainer.style.display = "";
    } else if (menu == "reef") {
        reefContainer.style.display = "";
        locationContainer.style.display = "";
        confirmReefContainer.style.display = "";
        confirmReefButton.innerText = "Choose Robot Alignment";
    } else if (menu == "screen") {
        screenChangeContainer.style.display = "";
        document.getElementById("msgDisplayer").innerText = screenmsg;
    } else if (menu == "choice") {
        choiceContainer.style.display = "";
    } else if (menu == "net") {
        netContainer.style.display = "";
    } else if (menu == "coral intake") {
        coralIntakeContainer.style.display = "";
    }
}

const reefChoice = document.getElementById("reefChoice");
const processorChoice = document.getElementById("processorChoice");
const netChoice = document.getElementById("netChoice");
const coralIntakeChoice = document.getElementById("coralIntakeChoice");

reefChoice.onclick = () => {
    changeMenu("reef");
}

processorChoice.onclick = () => {
    changeMenu("processor");
}

netChoice.onclick = () => {
    changeMenu("net");
}

coralIntakeChoice.onclick = () => {
    changeMenu("coral intake");
}

let reefScoreButtonClickable = true;
const resetReefStuff = () => {
    coralSelected = false;
    areaSelected = false;
    coralLevel = -1;
    document.getElementById("areaSelectionText").innerText = "Hover to the area you want to go to.";
    document.getElementById("coral").src = `coralLevelImages/coralNone.png`;
    document.getElementById("areaText").innerText = "Reef Area: None";
    document.getElementById("coralText").innerText = "Coral Level: None";
    document.getElementById("locationSelect").src = "locationSelectorImages/locationSelectorNone.png";
}

confirmReefButton.onclick = async () => {
    if(reefScoreButtonClickable && coralSelected && areaSelected) {
        reefScoreButtonClickable = false;
        confirmReefButton.innerText = "Scoring...";
        await scoreReef(reefArea, coralLevel);
        confirmReefButton.innerText = "Choose Robot Alignment";
        changeMenu("choice");
        reefScoreButtonClickable = true;
        resetReefStuff();
    }
}

changeMenu("none");