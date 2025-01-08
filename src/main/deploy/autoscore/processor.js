const processorButton = document.getElementById("button");
const animationIntervals = [];
const animationFPS = 11.25;
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
        if(frame == 15) {
            console.log("clearing");
            changeMenu("choice");
            clearInterval(animationInterval);
        } else {
            if(frame == 14) {
                processorImage.src = `processorEmpty.png`;
            }
            else {
                if(frame == 13) {
                    processorImage.src = `processorr12`;
                }
                else {
                    processorImage.src = `processorr${frame}.png`;
                }
            }
            frame++; 
        }
    }, 1000/animationFPS);
    console.log(animationInterval);
    animationIntervals.push(animationInterval);
}