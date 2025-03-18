const netButtonRed = document.getElementById("netScoreRed");
const netButtonBlue = document.getElementById("netScoreBlue");

let netClickable = true;
[netButtonRed, netButtonBlue].map(button => {
    button.onclick = async () => {
        if(netClickable) {
            netClickable = false;
            button.innerText = "Scoring...";
            await scoreNet(button.id == "netScoreRed");
            button.innerText = `Score Net ${button.id == "netScoreRed" ? " on Red Side" : " on Blue Side"}`;
            netClickable = true;
        }
    }
})