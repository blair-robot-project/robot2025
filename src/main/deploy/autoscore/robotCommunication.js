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

const scoreNet = async (isOnRedAllianceSide) => {
  console.log("scoring net");
  //communicate with network tables somehow
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
  console.log("await done");
}

const scoreProcessor = async () => {
  console.log("scoring processor");
  //communicate with network tables somehow
  await new Promise((resolve) => setTimeout(() => resolve(), 2000)); // Simulate asynchronous movement
  console.log("await done");
}

const intakeCoral = async (isAtTopSide) => {
  console.log("intaking coral");
  //communicate with network tables somehow
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
  console.log("await done");
}

const scoreReef = async (location, level) => {
  console.log(`Scoring on level: ${level} and location: ${location}`);
  //communicate with network tables somehow
  await new Promise((resolve) => setTimeout(() => resolve(), 1000)); // Simulate asynchronous movement
}

const getAlliance = async () => {
  let temporaryAlliance = "Red";
  if(temporaryAlliance == "Red") {
      document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
  } else if(temporaryAlliance == "Blue") {
      document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
  }
}

getAlliance();