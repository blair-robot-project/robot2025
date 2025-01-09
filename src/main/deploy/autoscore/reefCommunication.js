//robot coms part
const moveToReefLocation = async (location) => {
    // Call a function to move the robot to the specified reef location
    // Replace the following line with your actual implementation
    console.log(`Moving robot to ${location}`);
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 500)); // Simulate asynchronous movement
}

const scoreReefLevel = async (level) => {
    console.log(`Scoring on level: ${level}`);
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 500)); // Simulate asynchronous movement
}

const scoreReef = async (location, level) => {
    await moveToReefLocation(location);
    await scoreReefLevel(level);
}

const getAlliance = async () => {
    let alliance = "Red"; //temporary
    //get alliance from networktables
    if(alliance == "Red") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
    } else if(alliance == "Blue") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
    }
}

getAlliance();