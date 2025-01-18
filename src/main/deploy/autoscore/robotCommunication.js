const awaitCommandFinished = () => {
  return new Promise(resolve => {

  });
}

//I UNDERSTAND IT NOW

const scoreNet = async (isOnRedAllianceSide) => {
  if(isOnRedAllianceSide) {
    setCommand("netRed");
  } else {
    setCommand("netBlue");
  }
}

const scoreProcessor = async () => {
  console.log("scoring processor");
  setCommand("processor");
}

const intakeCoral = async (isAtTopSide) => {
  console.log("intaking coral");
  if(isAtTopSide) {
    setCommand("intakeCoralTop");
  } else {
    setCommand("intakeCoralBottom");
  }
}

const scoreReef = async (location, level) => {
  console.log(`Scoring on level: ${level} and location: ${location}`);
  setCommand(`l${level} location${location}`);
}

