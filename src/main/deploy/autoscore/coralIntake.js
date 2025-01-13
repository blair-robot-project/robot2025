let coralIntakeClickable = true;

const coralIntakeTop = document.getElementById("coralIntakeTop");
const coralIntakeBottom = document.getElementById("coralIntakeBottom");

[coralIntakeTop, coralIntakeBottom].map(button => {
    button.onclick = async () => {
        if(coralIntakeClickable) {
            coralIntakeClickable = false;
            button.innerText = "Intaking...";
            await intakeCoral(button.id == "coralIntakeTop");
            button.innerText = `Intaking Coral ${button.id == "coralIntakeTop" ? " on Top Side" : " on Bottom Side"}`;
            coralIntakeClickable = true;
            changeMenu("choice");
        }
    }
})