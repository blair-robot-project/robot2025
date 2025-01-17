//CODE FOR RUNNING ASYNCHRONOUS INTERVALS v
const asyncIntervals = [];

const runAsyncInterval = async (cb, interval, intervalIndex) => {
  await cb();
  if (asyncIntervals[intervalIndex]) {
    setTimeout(() => runAsyncInterval(cb, interval, intervalIndex), interval);
  }
};

const setAsyncInterval = (cb, interval) => {
  if (cb && typeof cb === "function") {
    const intervalIndex = asyncIntervals.length;
    asyncIntervals.push(true);
    runAsyncInterval(cb, interval, intervalIndex);
    return intervalIndex;
  } else {
    throw new Error('Callback must be a function');
  }
};

const clearAsyncInterval = (intervalIndex) => {
  if (asyncIntervals[intervalIndex]) {
    asyncIntervals[intervalIndex] = false;
  }
};
//CODE FOR RUNNING ASYNCRONOUS INTERVALS ^

const updateTime = 20; //ms
const networkTablePath = "../../../../networktables.json";

const awaitCommandFinished = () => {
  return new Promise(resolve => {

  });
}

const commandsTopic = "commands";
const numCheckTopic = "numCheck";
const numCheckToRobotTopic = "numCheckFromRobot";

const numToApp = "numToApp"
const numToRobot = "numToRobot"

const num = -1;

const scoreNet = async (isOnRedAllianceSide) => {
  console.log("scoring net");
  setTimeout(() => {
    client.addSample(commandsTopic, "netScored")
  }, 1000);
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
  console.log("await done");
}

const scoreProcessor = async () => {
  console.log("scoring processor");
  client.subscribe([commandsTopic, numCheckTopic], false, false, 0.02);
  client.publishNewTopic(commandsTopic, "string");
  client.publishNewTopic(numCheckTopic, "double");
  numCheck();
  client.connect();
  setTimeout(() => {
    client.addSample(commandsTopic, "processorScored")
  }, 1000);
  await new Promise((resolve) => setTimeout(() => resolve(), 2000)); // Simulate asynchronous movement
  console.log("await done");
}

const intakeCoral = async (isAtTopSide) => {
  console.log("intaking coral");
  setTimeout(() => {
    client.addSample(commandsTopic, "coralIntaken")
  }, 1000);
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
  console.log("await done");
}

const scoreReef = async (location, level) => {
  console.log(`Scoring on level: ${level} and location: ${location}`);

  setTimeout(() => {
    client.addSample(commandsTopic, "reefScored")
  }, 1000);
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
}

//const numCheck = async () => {
//  setTimeout(() => {
//    num =
//  }, 2000);
//}

let client = new NT4_Client(
  window.location.hostname, // window.location.hostname
  "autoscore", // topicAnnounceHandler
  (topic) => { // topicAnnounceHandler
    // Topic announce
  },
  (topic) => { // topicUnannounceHandler
    // Topic unannounce
  },
  (topic) => { // valueUpdateHandler
    // New data
  },
  () => { // onConnect
    // Connected
  },
  () => { // onDisconnect
    // Disconnected
  }
);

client.subscribeAllSamples(numToApp, false, false, 0.02);
//client.connect();
client.ws_connect();

//const numberGotten = client.getValue(numToApp, -1.0) / 2
const numberGotten = giveNumToAppValues()[-1] / 2
console.log("aaaaa");
console.log(numberGotten)
client.addSample(numToRobot, numberGotten)

window.addEventListener("load", () => {
      // Start NT connection
      client.subscribePeriodic([commandsTopic, numCheckTopic], false, false, 0.02);
      client.publishNewTopic(commandsTopic, "string");
      client.publishNewTopic(numCheckTopic, "int");
      //numCheck();
      //client.connect();
});