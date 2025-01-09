const scoreNet = async () => {
    console.log("scoring net");
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 3000)); // Simulate asynchronous movement
    console.log("await done");
}

const scoreProcessor = async () => {
    console.log("scoring processor");
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 3000)); // Simulate asynchronous movement
    console.log("await done");
}

const intakeCoral = async () => {
    console.log("intaking coral");
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 3000)); // Simulate asynchronous movement
    console.log("await done");
}

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
    //wait half a second if network tables is bad
    await new Promise((resolve) => setTimeout(() => resolve(), 500));
    let alliance = "Red"; //temporary
    //get alliance from networktables
    let response = await fetch("../../../../networktables.json");
    let json = response.json();
    console.log("json", json);
    if(alliance == "Red") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
    } else if(alliance == "Blue") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
    }
}

getAlliance();