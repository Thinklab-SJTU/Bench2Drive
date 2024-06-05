#!/usr/bin/env groovy


String CARLA_HOST 
String CARLA_RELEASE
String TEST_HOST
String COMMIT
String ECR_REPOSITORY = "456841689987.dkr.ecr.eu-west-3.amazonaws.com/scenario_runner"
boolean CARLA_RUNNING = false
boolean CONCURRENCY = true

// V3 - include detection of concurrent builds

pipeline
{
    agent none

    options
    {
        buildDiscarder(logRotator(numToKeepStr: '3', artifactNumToKeepStr: '3'))
        skipDefaultCheckout()
    }

    stages
    {
        stage('setup')
        {
            agent { label "master" }
            steps
            {
                checkout scm
                script
                {
                    jenkinsLib = load("/home/jenkins/scenario_runner.groovy")
                    TEST_HOST = jenkinsLib.getUbuntuTestNodeHost()
                    CARLA_HOST= sh(
                        script: "cat ./CARLA_VER | grep HOST | sed 's/HOST\\s*=\\s*//g'",
                        returnStdout: true).trim()
                    CARLA_RELEASE = sh(
                        script: "cat ./CARLA_VER | grep RELEASE | sed 's/RELEASE\\s*=\\s*//g'",
                        returnStdout: true).trim()
                    COMMIT = sh(returnStdout: true, script: "git log -n 1 --pretty=format:'%h'").trim()
                }
                println "using CARLA version ${CARLA_RELEASE} from ${TEST_HOST}"
            }
        }
        stage('get concurrency status')
        {
            options
            {
                lock resource: 'ubuntu_gpu', skipIfLocked: true
            }
            agent { label "master" }
            steps
            {
                script
                {
                    CONCURRENCY = false
                    println "no concurrent builds detected."
                }
            }
        }
        stage('act on concurrency')
        {
            agent { label "master" }
            steps
            {
                script
                {
                    if ( CONCURRENCY == true )
                    {
                        println "concurrent builds detected, prebuilding SR image."
                        stage('prebuild SR docker image')
                        {
                            //checkout scm
                            sh "docker build -t jenkins/scenario_runner:${COMMIT} ."
                            sh "docker tag jenkins/scenario_runner:${COMMIT} ${ECR_REPOSITORY}:${COMMIT}"
                            sh '$(aws ecr get-login | sed \'s/ -e none//g\' )' 
                            sh "docker push ${ECR_REPOSITORY}"
			    sh "docker image rmi -f \"\$(docker images -q ${ECR_REPOSITORY}:${COMMIT})\""
                        }
                    }
                }
            }
        }
        stage('lock ubuntu_gpu instance')
        {
            options
            {
                lock resource: "ubuntu_gpu"
            }
            stages
            {
                stage('start server')
                {
                        agent { label "master" }
                        steps
                        {
                            script
                            {
                                jenkinsLib = load("/home/jenkins/scenario_runner.groovy")
                                jenkinsLib.StartUbuntuTestNode()
                            }
                        }
                }
                stage('deploy')
                {
                        parallel
                        {
                            stage('build SR docker image')
                            {
                                    agent { label "master" }
                                    steps
                                    {
                                        script
                                        {
                                            if ( CONCURRENCY == false )
                                            {
                                                //checkout scm
                                                sh "docker build -t jenkins/scenario_runner:${COMMIT} ."
                                                sh "docker tag jenkins/scenario_runner:${COMMIT} ${ECR_REPOSITORY}:${COMMIT}"
                                                sh '$(aws ecr get-login | sed \'s/ -e none//g\' )'
                                                sh "docker push ${ECR_REPOSITORY}"
						sh "docker image rmi -f \"\$(docker images -q ${ECR_REPOSITORY}:${COMMIT})\""
                                            }
                                            else
                                            {
                                                println "SR docker image already built due concurrency"
                                            }
                                        }
                                    }
                            }
                            stage('deploy CARLA')
                            {
                                    stages
                                    {
                                        stage('install CARLA')
                                        {
                                                agent { label "secondary && ubuntu && gpu && sr" }
                                                steps
                                                {
                                                    println "using CARLA version ${CARLA_RELEASE}"
                                                    sh "wget -qO- ${CARLA_HOST}/${CARLA_RELEASE}.tar.gz | tar -xzv -C ."
                                                }
                                        }
                                    }
                            }
                        }
                }
                stage('run test')
                {
                    agent { label "secondary && ubuntu && gpu && sr" }
                        steps
                        {
                            sh 'DISPLAY= ./CarlaUE4.sh -opengl -nosound > CarlaUE4.log&'
                        sleep 10
                            script
                            {
                                    sh '$(aws ecr get-login | sed \'s/ -e none//g\' )' 
                                    sh "docker pull ${ECR_REPOSITORY}:${COMMIT}"
                                    sh "docker container run --rm --network host -e LANG=C.UTF-8 \"${ECR_REPOSITORY}:${COMMIT}\" -c \"python3 scenario_runner.py --scenario FollowLeadingVehicle_1 --debug --output --reloadWorld \""
                                    deleteDir()
                            }
                        }
                }
            }
            post
            {
                always
                {
                        node('master')
                        {
                            script  
                            {
                                    jenkinsLib = load("/home/jenkins/scenario_runner.groovy")
                                    jenkinsLib.StopUbuntuTestNode()
                                    echo 'test node stopped'
                                    sh 'docker system prune --volumes -f'
                            }
                            deleteDir()
                        }
                }
            }
        }
    }
}
