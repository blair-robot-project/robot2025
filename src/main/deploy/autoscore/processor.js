const processorButton = document.getElementById("processorScore");
const animationIntervals = [];
const animationFPS = 11.25;
const processorImage = document.getElementById("processor");
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
        processorImage.style.filter = "brightness(1)";
        animationIntervals.map((animation) => {
            clearInterval(animation);
        });
        let frame = 1;
        const animationInterval = setInterval(() => {
            //increase the width because the animation shrinks the img for some reason
            if(frame == 15) {
                console.log("clearing");
                clearInterval(animationInterval);
            } else {
                if(frame == 14) {
                    processorImage.src = `processorImages/processorEmpty.png`;
                    //put it back
                }
                else {
                    if(frame == 13) {
                        processorImage.src = `processorImages/processor12.png`;
                    }
                    else {
                        processorImage.src = `processorImages/processor${frame}.png`;
                    }
                }
                frame++; 
            }
        }, 1000/animationFPS);
        animationIntervals.push(animationInterval);
        console.log("calling score processor");
        processorButton.innerText = "Scoring...";
        await scoreProcessor();
        changeMenu("choice");
        processorButton.innerText = "Score Processor";
        processorClickable = true;
    }
}