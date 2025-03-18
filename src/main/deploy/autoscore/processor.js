const processorButton = document.getElementById("processorScore");
let processorClickable = true;
processorButton.onmouseenter = () => {
    processorImage.style.filter = "brightness(1.25)";
}

processorButton.onmouseleave = () => {
    processorImage.style.filter = "brightness(1)";
}
processorButton.onclick = async () => {
    if(processorClickable) {
        processorClickable = false;
        processorButton.innerText = "Scoring Processor...";
        await scoreProcessor();
        processorButton.innerText = "Score Processor";
        processorClickable = true;
    }
}