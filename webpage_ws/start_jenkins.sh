# Exit on any error. If something fails, there's no use
# proceeding further because Jenkins might not be able 
# to run in that case
set -e

# Set the JENKINS_HOME environment variable. This will cause
# Jenkins will run from this directory.
# Create the JENKINS_HOME directory if it doesn't exist.
export JENKINS_HOME=~/webpage_ws/jenkins
mkdir -p $JENKINS_HOME

# Install java. We are using JRE 17.
sudo apt-get update -y || true
sudo apt-get install -y openjdk-17-jre

# Download the Jenkins .war file, if not there already
cd ~
JENKINS_FILE="/home/user/jenkins.war"
if [ ! -f "$JENKINS_FILE" ]; then
    wget https://updates.jenkins.io/download/war/2.430/jenkins.war
fi

# Jenkins is about to run, but we must check if 
# it's is already running
string=`ps ax | grep jenkins`
if [[ $string == *"jenkins.war"* ]]; then
    # Don't proceed further. Jenkings is already running
    echo "Jenkins is running already. Exiting."
    exit 0
else
    # Start Jenkins, since it's not running yet. 
    # Run Jenkins with a prefix. A prefix is needed because we are using a 
    # reverse proxy to run it on the Academy. This may not be necessary in your setup.
    # Store Jenkins proceess ID in JENKINS_PID
    java -jar jenkins.war --prefix="/$SLOT_PREFIX/jenkins/" &
    JENKINS_PID=$!
    sleep 15s

    # Calculate the Jenkins proxy address
    # This is NOT required for running Jenkins. It's just something we
    # need to do on the Academy. INSTANCE_ID and SLOT_PREFIX are Academy variables.
    INSTANCE_ID=`curl -s http://169.254.169.254/latest/meta-data/instance-id`
    URL=`echo "https://$INSTANCE_ID.robotigniteacademy.com/$SLOT_PREFIX/jenkins/"`
    echo ""
    echo "1. Jenkins is running in the background."
    echo "2. Switch to the webpage tab to see it."

    # Create a file that will store vital information
    # about the Jenkins instance
    # Save the running instance info to the file
    # This is just for your convenience
    STATE_FILE=~/jenkins__pid__url.txt
    touch $STATE_FILE
    echo "To stop Jenkins, run:" > $STATE_FILE
    echo "kill $JENKINS_PID" >> $STATE_FILE
    echo "" >> $STATE_FILE
    echo "Jenkins URL: " >> $STATE_FILE
    echo $URL >> $STATE_FILE
    echo "3. See '$STATE_FILE' for Jenkins PID and URL."

    # Start the ROS script that TAKES OFF the drone in the background
    # Capture the process ID of that script in TAKEOFF_ID
    # Wait for 15 seconds and then kill the process
    rostopic pub /drone/takeoff std_msgs/Empty '{}' &
    TAKEOFF_ID=$!
    sleep 15s
    kill $TAKEOFF_ID

    # Start the ROS script that LANDS the drone
    # Capture the process ID of that script in LAND_ID
    # Wait for 5 seconds and then kill the process
    rostopic pub /drone/land std_msgs/Empty '{}' &
    LAND_ID=$!
    sleep 5s
    kill $LAND_ID
fi