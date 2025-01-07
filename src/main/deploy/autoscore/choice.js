const reefChoice = document.getElementById("reefChoice");
const processorChoice = document.getElementById("processorChoice");

reefChoice.onclick = () => {
    changeMenu("location pick");
}

processorChoice.onclick = () => {
    changeMenu("processor");
}