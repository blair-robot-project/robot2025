const coralIntakeButton = document.getElementById("coralIntake");

let coralIntakeClickable = true;
coralIntakeButton.onclick = async () => {
    if(coralIntakeClickable) {
        coralIntakeClickable = false;
        coralIntakeButton.innerText = "Scoring...";
        await scoreNet();
        coralIntakeButton.innerText = "Score Net";
        coralIntakeClickable = true;
        changeMenu("choice");
    }
}