export class LLMChat {
    constructor(rosConnection, divID, chatHistoryID, runOnBotID) {
        this.ros = rosConnection
        this.divID = divID;
        this.chatHistoryID = chatHistoryID;
        this.runOnBotID = runOnBotID;

        this.llmListener = new ROSLIB.Topic({
            ros : this.ros,
            name : '/llm_response',
            messageType : 'std_msgs/String'
        });

        this.llmListener.subscribe((message) => this.llmListenerCallback(message));

        this.userResponsePub = new ROSLIB.Topic({
            ros : this.ros,
            name : '/user_response',
            messageType : 'std_msgs/String'
        });

    }

    changeButtonVisibility(showButton) {
        let visibiity = "none";
        if (showButton) visibiity = "visible"
        document.getElementById(this.runOnBotID).style.display = visibiity;
    }

    llmListenerCallback(message) {
        
        let historyElem = document.getElementById(this.chatHistoryID);
        const text = document.createElement("pre");
        text.textContent = `AGENT:\n${message.data}`;
        historyElem.appendChild(text);
        this.changeButtonVisibility(false);

    }
    

    userInputCallback(inputID) {
        this.changeButtonVisibility(false);
        let elem = document.getElementById(inputID);
        let value = elem.value;
        
        let str = new ROSLIB.Message({data : value});
        this.userResponsePub.publish(str);

        let historyElem = document.getElementById(this.chatHistoryID);
        const text = document.createElement("div");
        text.textContent = `USER: ${value}`;
        historyElem.appendChild(text);

    }

  }
