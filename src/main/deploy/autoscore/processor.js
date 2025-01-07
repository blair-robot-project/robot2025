const processorButton = document.getElementById("button");
const animationIntervals = [];
const animationFPS = 6;
const processorImage = document.getElementById("processor");

processorButton.onmouseenter = () => {
        processorImage.style.filter = "brightness(1.25)";
    }

processorButton.onmouseleave = () => {
    processorImage.style.filter = "brightness(1)";
}

processorButton.onclick = () => {
    scoreProcessor();
    processorImage.style.filter = "brightness(1)";
    animationIntervals.map((animation) => {
        clearInterval(animation);
    })
    let frame = 1;
    const animationInterval = setInterval(() => {
        if(frame == 8) {
            console.log("clearing");
            changeMenu("choice");
            clearInterval(animationInterval);
        } else {
            if(frame == 7) {
                processorImage.src = `processor1.png`;
            } else {
                processorImage.src = `processor${frame}.png`;
            }
            frame++; 
        }
    }, 1000/animationFPS);
    console.log(animationInterval);
    animationIntervals.push(animationInterval);
}