const reefContainer = document.querySelector(".reefContainer");
const processorContainer = document.querySelector(".processorContainer");
const locationContainer = document.querySelector(".locationContainer");
const screenChangeContainer = document.querySelector(".screenChangeContainer");
const choiceContainer = document.querySelector(".choiceContainer");

let menu;

const changeMenu = (menuVal, screenmsg) => {
    menu = menuVal;
    reefContainer.style.display = "none";
    locationContainer.style.display = "none";
    screenChangeContainer.style.display = "none";
    processorContainer.style.display = "none";
    choiceContainer.style.display = "none";
    if (menu == "processor") {
        processorContainer.style.display = "";
    } else if (menu == "reef"){
        reefContainer.style.display = "";
    } else if (menu == "location pick") {
        locationContainer.style.display = "";
    } else if (menu == "screen change") {
        screenChangeContainer.style.display = "";
        document.getElementById("msgDisplayer").innerText = screenmsg;
    } else if (menu == "choice") {
        choiceContainer.style.display = "";
    }
}

changeMenu("choice");
