const scoreNet = async () => {
    console.log("scoring net");
    //communicate with network tables somehow
    await new Promise((resolve) => setTimeout(() => resolve(), 3000)); // Simulate asynchronous movement
    console.log("await done");
}