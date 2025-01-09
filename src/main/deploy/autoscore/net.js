const netButton = document.getElementById("netScore");

let netClickable = true;
netButton.onclick = async () => {
    if(netClickable) {
        netClickable = false;
        netButton.innerText = "Scoring...";
        await scoreNet();
        netButton.innerText = "Score Net";
        netClickable = true;
        changeMenu("choice");
    }
}