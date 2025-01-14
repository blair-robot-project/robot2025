//Instantiate the new client
// using `window.location.hostname` causes the client to open a 
// NT connection on the same machine as is serving the website.
// It could be hardcoded to point at a roboRIO if needed.
// var nt4Client = new NT4_Client(
//     window.location.hostname, 
//     topicAnnounceHandler,
//     topicUnannounceHandler,
//     valueUpdateHandler,
//     onConnect,
//     onDisconnect
// );
//nt4client IMPORTANT methods
/*
subscribeTopicNames : list of topics name to subscribe to 
subscribePeriodic : list of topic names, data rate in seconds
addSample : add a sample value to a topic
publishTopic : name of topic, type
unSubscribe
clearAllSubscriptions
 */


// Allocate a variable to hold the subscription to all topics
var subscription = null;

/**
 * Topic Announce Handler
 * The NT4_Client will call this function whenever the server announces a new topic.
 * It's the user's job to react to the new topic in some useful way.
 * @param {NT4_Topic} newTopic The new topic the server just announced.
 */
function topicAnnounceHandler( newTopic ) {

    //For this example, when a new topic is announced,
    console.log("new topic id: " + newTopic.id.toString());
    console.log("new topic name: " + newTopic.name.toString());
    console.log("new topic type: " + newTopic.type.toString());
    console.log("new topic properties: " + newTopic.getPropertiesString());

}

/**
 * Topic UnAnnounce Handler
 * The NT4_Client will call this function whenever the server un-announces a topic.
 * It's the user's job to react to the un-anncouncement in some useful way.
 * @param {NT4_Topic} removedTopic The topic the server just un-announced.
 */
function topicUnannounceHandler( removedTopic ) {
    //For this example, when a topic is unannounced, we remove its row.
    console.log("removed topic name: " + removedTopic.name.toString());
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
    console.log(topicAnnounceHandler(topic));
    console.log(timestamp_us);
    console.log(value);
}

/**
 * On Connection Handler
 * The NT4_Client will call this after every time it successfully connects to an NT4 server.
 */
function onConnect() {

    console.log("robot connected")
    
}

/**
 * On Disconnection Handler
 * The NT4_Client will call this after every time it disconnects to an NT4 server.
 */
function onDisconnect() {
    // //For this example, we simply mark the status as disconnected.
    // console.log("robot disconnected")

    // //throwing an error to stop the reconnect cycle because we have nothing planned currently
    // throw new Error("Robot disconnected");
    // //Since we've disconnected from the server, the connection is no longer valid.
    // subscription = null;
}

const getAlliance = () => {
    let temporaryAlliance = "Red";
    if(temporaryAlliance == "Red") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
    } else if(temporaryAlliance == "Blue") {
        document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
    }
}
changeMenu(menuChoices.CHOICE);
  
