const { SSL_OP_EPHEMERAL_RSA } = require('constants');

document.onkeydown = updateKey;
document.onkeyup = resetKey;

var server_port = 65432;
var server_addr = "192.168.1.180";   // the IP address of your Raspberry PI


var nodeConsole = require('console')
var myConsole = new nodeConsole.Console(process.stdout,process.stderr)


function update_recvd_data(data) {
    myConsole.log(data.toString());
    data_str = data.toString()
    var recvd_data_count = 0;
    var message = "";
    while (1)
    {
        
        delimeter_index = data_str.indexOf('\n');
        myConsole.log("delimeter indexxxx",delimeter_index);
        if ( delimeter_index !== -1 ) {

            message = data_str.slice(0,delimeter_index);
            myConsole.log("message",message);
            data_str = data_str.replace(message+'\n',"");
            myConsole.log("data after slice",data_str);
            recvd_data_count += 1;
        

            if (recvd_data_count === 1)
            {
                document.getElementById("cpu_usage").innerHTML = message;
            }   
            if (recvd_data_count === 2)
            {
                document.getElementById("cpu_temperature").innerHTML = message;
            }
            if (recvd_data_count === 3)
            {
                document.getElementById("gpu_temperature").innerHTML = message;
            }
            if (recvd_data_count === 4)
            {
                document.getElementById("disk_space").innerHTML = message;
            }
            if (recvd_data_count === 5)
            {
                document.getElementById("ram_info").innerHTML = message;
            }
            if (recvd_data_count === 6)
            {
                document.getElementById("ip_pi").innerHTML = message;
            }
            if (recvd_data_count === 7)
            {
                document.getElementById("power").innerHTML = message;
            }
        }
        else
        {
            break;
        }
    }
}
function client(){
    
    const net = require('net');
    var input = "getData"
    const client = net.createConnection({ port: server_port, host: server_addr }, () => {
        // 'connect' listener.
        myConsole.log('connected to server!');
        // send the message
        client.write(`${input}\r\n`);
    });
    
    // get the data from the server
    client.on('data', function(data) {
        update_recvd_data(data);
    });

    client.on('end', () => {
        myConsole.log('disconnected from server');
    });


}
function send_data(input){
    const net = require('net');
 
    const client = net.createConnection({ port: server_port, host: server_addr }, () => {
        // 'connect' listener.
        myConsole.log('connected to server!');
        // send the message
        client.write(`${input}`);
    });
}
// for detecting which key is been pressed w,a,s,d
function updateKey(e) {

    e = e || window.event;

    myConsole.log(e.keyCode);

    if (e.keyCode == '87') {
        // up (w)
        document.getElementById("upArrow").style.color = "darkred";
        send_data("87");
    }
    else if (e.keyCode == '83') {
        // down (s)
        document.getElementById("downArrow").style.color = "darkred";
        send_data("83");
    }
    else if (e.keyCode == '65') {
        // left (a)
        document.getElementById("leftArrow").style.color = "darkred";
        send_data("65");
    }
    else if (e.keyCode == '68') {
        // right (d)
        document.getElementById("rightArrow").style.color = "darkred";
        send_data("68");
    }
    else if (e.keyCode == '81') {
        // To stop(q)
        send_data("81");
    }
}

// reset the key to the start state 
function resetKey(e) {

    e = e || window.event;

    document.getElementById("upArrow").style.color = "yellowgreen";
    document.getElementById("downArrow").style.color = "yellowgreen";
    document.getElementById("leftArrow").style.color = "yellowgreen";
    document.getElementById("rightArrow").style.color = "yellowgreen";
}


// update data for every 1000ms
function update_data(){
    setInterval(function(){
        // get image from python server
        client();
    }, 1000);
}
