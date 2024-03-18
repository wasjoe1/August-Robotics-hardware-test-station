// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickBtn(element) {
    const buttonDict = {
        "scanBtn": "SCAN",
        "saveBtn": "SAVE",
        "connectBtn": "CONNECT",
        "closeBtn": "CLOSE"
    }
    executeCommand(createCmdData(buttonDict[element.id]))    
}

// ------------------------------------------------------------------------------------------------
// Functions
function createCmdData(buttonString) {
    var inputArr = []
    if (buttonString == "SAVE") { // if not saved, input arr inputs will be undefined
        inputArr = [
            document.getElementById("imuInput1").value,
            document.getElementById("imuInput2").value,
            document.getElementById("imuInput3").value,
            document.getElementById("imuInput4").value,
        ]
    }

    return {
        button: buttonString,
        parameter1: inputArr[0],
        parameter2: inputArr[1],
        parameter3: inputArr[2],
        parameter4: inputArr[3],
    }
}

function executeCommand(cmd) {
    console.log(cmd)

    // var cmd_dict = {}
    // cmd_dict[current_step] = cmd
    // cmd_str = JSON.stringify(cmd_dict)
    // // cmd_str = current_step + "_" + cmd
    // console.log("send cmd: " + cmd_str)
    // var url = "http://" + ip_addr + "/command/" + cmd_str
    // var request = new XMLHttpRequest()
    // request.open("GET", url)
    // request.send()
}