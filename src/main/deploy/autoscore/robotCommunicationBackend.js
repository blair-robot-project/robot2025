/**
 *   outlineViewer.js - Simple example of using the JS network 
 *   tables interface to produce an Outline Viewer style display 
 *   of all NT values.
 */


// Import the nt4 client as a module

//Instantiate the new client
// using `window.location.hostname` causes the client to open a 
// NT connection on the same machine as is serving the website.
// It could be hardcoded to point at a roboRIO if needed.
const nt4Client = new NT4_Client(
    window.location.hostname, 
    topicAnnounceHandler,
    topicUnannounceHandler,
    valueUpdateHandler,
    parameterChange,
    onConnect,
    onDisconnect
);

// Grab a reference to the HTML object where we will put values
var table = document.getElementById("mainTable");

// Create a map to remember the topics as they are announced,
// and what HTML table cell they should populate
var cellTopicIDMap = new Map();

// Allocate a variable to hold the subscription to all topics
var subscription = null;

/**
 * Topic Announce Handler
 * The NT4_Client will call this function whenever the server announces a new topic.
 * It's the user's job to react to the new topic in some useful way.
 * @param {NT4_Topic} newTopic The new topic the server just announced.
 */
function topicAnnounceHandler( newTopic ) {
    console.log("topic announced: " + newTopic.name);

}

/**
 * Topic UnAnnounce Handler
 * The NT4_Client will call this function whenever the server un-announces a topic.
 * It's the user's job to react to the un-anncouncement in some useful way.
 * @param {NT4_Topic} removedTopic The topic the server just un-announced.
 */
function topicUnannounceHandler( removedTopic ) {
}

/**
 * Value Update Handler
 * The NT4_Client will call this function whenever the server sends a value update
 * for a topic.
 * @param {NT4_Topic} topic The topic with a value update
 * @param {double} timestamp_us The server time of the value update occurring
 * @param {*} value the new value for the topic
 */
function valueUpdateHandler( topic, timestamp_us, value ) {
    console.log("value updated: " + topic.name);
    if(topic.name == "/webcom/Alliance") {
        setAlliance(value);
    }
}

/**
 * On Connection Handler
 * The NT4_Client will call this after every time it successfully connects to an NT4 server.
 */
function onConnect() {
    console.log("connected to robot");
    subscription = nt4Client.subscribePeriodic(["/webcom"], 0.1);
    //Finally, update status to show we've connected.
    changeMenu(menuChoices.CHOICE);
}

/**
 * On Disconnection Handler
 * The NT4_Client will call this after every time it disconnects to an NT4 server.
 */
function onDisconnect() {
    console.log("disconnected from robot");
    //For this example, we simply mark the status as disconnected.
    changeMenu(menuChoices.SCREEN, "NO ROBOT CONNECTION");

    //Since we've disconnected from the server, the connection is no longer valid.
    subscription = null;
}

function parameterChange(changeObject) {
    //gives an object with the change
    
}

const setAlliance = (alliance) => {
    console.log(alliance);
    if(alliance == "Red") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
    } else if(alliance == "Blue") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
    }
}


const publishCommand = (commandName) => {

}

const commandPath = "/webcom/Command";
const isDonePath = "/webcom/isDone";

const commandPublishTopic = nt4Client.publishNewTopic(commandPath, NT4_TYPESTR.STR);

setCommand = (command) => {
    nt4Client.addSample(commandPath, nt4Client.getServerTime_us(), command);
}