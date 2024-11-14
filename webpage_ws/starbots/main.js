let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        
        ros: null,
        rosbridge_address: 'wss://i-0d54e7b887b69a36a.robotigniteacademy.com/82601360-67e9-4635-b24f-afa99f653e13/rosbridge/',
        connected: false,
        // page content
        menu_title: 'Connection',
        main_title: 'Main title, from Vue!!',
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                this.setCamera()
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
                this.connect();
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
                document.getElementById('divCamera').innerHTML = ''
                this.connect();

            })
        },
        disconnect: function() {
            this.ros.close()
        },
        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 320,
                height: 240,
                topic: 'website/color/image',
                ssl: true,
            })
        },
    },
    mounted() {
        // page is ready
        console.log('page is ready!')
        this.connect();
    },
})