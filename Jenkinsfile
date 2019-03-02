node {
    try {
        stage('Pre-build') {
            checkout scm
            sh 'git submodule update --init --recursive'
        }

        stage 'Build'
        def image = docker.build "fsae-2020-${BUILD_TAG.toLowerCase()}"

//        stage('Lint') {
//            image.inside {
//                sh './lintit.sh'
//            }
//        }

        stage('Test') {
            image.inside {
                sh '''
                . /robot/devel/setup.sh
                catkin_make
                catkin_make run_tests
                '''
            }
        }

//        stage('Build Teensy') {
//            image.inside {
//                sh '''
//                . /opt/magellan-deps/devel/setup.sh
//                . /robot/devel/setup.sh
//                /robot/src/magellan_firmware/compile.sh
//                '''
//            }
//        }
    }
    catch (ex) {
        currentBuild.result = 'FAILURE'
    }
    finally {
        stage('Cleanup') {
            cleanWs()
            sh "docker rmi fsae-2020-${BUILD_TAG.toLowerCase()}"
        }
    }
}
