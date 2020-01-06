pipeline {
  agent none
  options{
    skipDefaultCheckout()
  }
  stages {
    stage('Checkout'){
      agent{
        label 'noida-linux-ubuntu16-ci-slave'
      }
      steps{
        checkout scm
      }
    }
    stage('Setup Environment') {
      agent{
        label 'noida-linux-ubuntu16-ci-slave'
      }
      steps {
        sh './deps/install-deps.sh'
      }
    }
    stage('Build') {
      agent{
        label 'noida-linux-ubuntu16-ci-slave'
      }
      steps {
        sh 'make deviceOSWD-dummy-cross-debug'
      }
    }
    
    //stage('Test and Code Review') {
      //parallel {
        stage('Test'){
          agent{
            label 'noida-linux-ubuntu16-ci-slave'
          }
          steps {
            sh 'touch coverage.xml'
            sh 'touch report.xml'
          }
        }
        
        /*stage('SonarQube'){
          agent{
            label 'master'
          }
          environment {
            scannerHome = tool 'SonarQubeScanner'
          }
          steps {
            sh 'cppcheck --xml --xml-version=2 --enable=all *[^"deps"] 2> cppcheck-report.xml'
            withSonarQubeEnv('sonarqube') {
              sh "${scannerHome}/bin/sonar-scanner"
            }
          }
        }*/
      //}
    //}
    stage('Auto Doc'){
      agent{
        label 'noida-linux-ubuntu16-ci-slave'
      }
      steps{
        sh 'doxygen -g'
        sh 'doxygen Doxyfile'
      }
    }
  }
 post{
   success{
      slackSend(channel: '#edge-jenkins-ci', color: 'good', message: "JOB NAME: ${env.JOB_NAME}\nBUILD NUMBER: ${env.BUILD_NUMBER}\nSTATUS: ${currentBuild.currentResult}\n${env.RUN_DISPLAY_URL}")
    }
    failure{
      slackSend(channel: '#edge-jenkins-ci', color: 'danger', message: "JOB NAME: ${env.JOB_NAME}\nBUILD NUMBER: ${env.BUILD_NUMBER}\nSTATUS: ${currentBuild.currentResult}\n${env.RUN_DISPLAY_URL}")
    }
    unstable{
      slackSend(channel: '#edge-jenkins-ci', color: 'warning', message: "JOB NAME: ${env.JOB_NAME}\nBUILD NUMBER: ${env.BUILD_NUMBER}\nSTATUS: ${currentBuild.currentResult}\n${env.RUN_DISPLAY_URL}")
    }
    always{
      node('noida-linux-ubuntu16-ci-slave'){
        junit 'report.xml'
        archiveArtifacts artifacts: 'html/**/*'
        step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: 'coverage.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
        //sh "curl -X POST -H \"Content-type: application/json\" --data '{\"channel\": \"#edge-jenkins-ci\", \"username\": \"webhookbot\", \"text\": \"JOB NAME: ${env.JOB_NAME}\\nBUILD NUMBER: ${env.BUILD_NUMBER}\\nSTATUS: ${currentBuild.currentResult}\\n${env.RUN_DISPLAY_URL}\"}' https://hooks.slack.com/services/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"
      }
    }
 }
}
