const levelOne = document.getElementById("L1");
const levelTwo = document.getElementById("L2");
const levelThree = document.getElementById("L3");
const levelFour = document.getElementById("L4");

const levelList = [levelOne, levelTwo, levelThree, levelFour];

const coralText = document.getElementById("coralText");
levelList.map(level => {
    level.onmouseenter = () => {
        if(!coralSelected) {
            document.getElementById("coral").src = `coralLevelImages/coral${level.id}.png`;
        }
    }
    level.onmouseleave = () => {
        if(!coralSelected) {
            document.getElementById("coral").src = `coralLevelImages/coralNone.png`;
        }
    }
    level.onclick = () => {
        coralSelected = true;
        document.getElementById("coral").src = `coralLevelImages/coral${level.id}.png`;
        coralLevel = parseInt(level.id.slice(1));
        coralText.innerText = `Coral Level: ${coralLevel}`;
        if(areaSelected) {
            confirmReefButton.innerText = `Score at Level ${level.id} and Area ${reefArea}`;
        }
    }
});


