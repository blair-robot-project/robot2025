const levelOne = document.getElementById("L1");
const levelTwo = document.getElementById("L2");
const levelThree = document.getElementById("L3");
const levelFour = document.getElementById("L4");

const levelList = [levelOne, levelTwo, levelThree, levelFour];
let reefLevel = 1;

levelList.map(level => {
    level.onmouseenter = () => {
        document.getElementById("coral").src = `coral${level.id}.png`;
    }
    level.onmouseleave = () => {
        document.getElementById("coral").src = `coralNone.png`;
    }
    level.onclick = () => {
        reefLevel = parseInt(level.id.slice(1));
        scoreReefLevel(reefLevel);
        changeMenu("choice");
    }
});
