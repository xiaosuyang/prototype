#include "cmdpanel.h"

template<typename T>
inline T max(const T a, const T b){
	return (a > b ? a : b);
}

template<typename T>
inline T min(const T a, const T b){
	return (a < b ? a : b);
}

KeyBoard::KeyBoard(){


    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );
    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    // case ' ':
    //     return UserCommand::EXIT;
    case '1':
        return L2_B;
    case '2':
        return L2_A;
    case '3':
        return L2_X;
    case '4':
        return START;
    case '5':
        return L2_Y;
    case '0':
        return L1_X;
    case '9':
        return L1_A;
    case '8':
        return L1_Y;
    case ' ':
        return NONE;
    default:
        return NONE;
    }
}
void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        uservalue.direction+=sensitivityUp;
        break;
    case 's':case 'S':
        uservalue.direction-=sensitivityUp;
        break;
    case 't':case 'T':
         uservalue.Starttime=true;
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
}

void* KeyBoard::run(void *arg){
    while(1){
        // std::cout << "key test" << std::endl;
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == NONE)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
}


