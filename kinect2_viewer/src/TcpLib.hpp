/**
  @file: TcpLib.hpp
  
  @brief:Linux用TCP通信サーバ用ライブラリ
          ユーザが使用するクラス
          1.TcpServer サーバ用
          2.TcpClient クライアント用
  @note TcpMasterクラスは、サーバとクライアントの
        共通変数・関数用クラスなので
        単独では使用しないでください。
        使用方法は、各クラスのヘッダコメントに書いてあります。
  
  @author: Atsushi Sakai 
*/

#ifndef TCP_LIB_HPP
#define TCP_LIB_HPP

#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <sys/socket.h>   //ソケット通信用
#include <errno.h>        //エラー解析用
#include <netinet/in.h>   //sockaddr_in用
#include <arpa/inet.h>    //inet_addr用

using namespace std;
/** 
 * @brief TCP通信用マスタクラス
 *        サーバとクライアントの共通の関数や変数用
 *        ReadとWrite関数はこちらのクラスに書いてあるので注意
 */
class TcpMaster{
  public:
    TcpMaster(){
      InitMember();//メンバ変数の初期化
    }

    virtual ~TcpMaster(){
      close(selfSock_);//ソケットを閉じる
    }

    //=====メンバ変数=====
    bool isInit_;   //初期化されたかどうか？
    int selfSock_;  //自身のソケット
    int clientSock_;//クライアントのソケット
    //(クライアントクラスの場合 selfsock_=clientsock_)
    int port_;      //ポート番号
    
    //=====メンバ関数=====
    /**
     *  @brief データ受信関数
     *  @param data 受信データ
     *  @return 受信データのバイト数
     */
    int Read(vector<char> &data){
      if(!isInit_){//初期化チェック
        cout<<"[Read] This object doesn't init"<<endl;
        return 0;
      }

      //データの受信
      const int MAX_READ_BUF=22800;
      char RecBuf[MAX_READ_BUF];//受信バッファ

      //データの受信
      int nRead=read(selfSock_, RecBuf,sizeof(RecBuf));

      //受信データのダイアグノシス
      if(nRead<=0){
        cout<<"[Read] cannot receive packet"<<endl;
	perror("Read");
        data.clear();//受信変数を空にする
        return 0;
      }
      else if(nRead>=MAX_READ_BUF){
        cout<<"[Read] Buffer Overflow:"<<nRead<<endl;
        data.clear();//受信変数を空にする
        return 0;
      }

      //vectorに格納
      data.resize(nRead);//事前確保 reserveではない
      for(int ir=0;ir<nRead;ir++){
        data[ir]=RecBuf[ir];
      }

      return nRead;//受信数を返す
    }

    /**
     *  @brief データ送信関数
     *  @param data 送信データ
     *  @return 送信データのバイト数
     */
    int Write(const vector<char> &data) const {
      if(!isInit_){//初期化チェック
        cout<<"[Write] This object doesn't init"<<endl;
        return 0;//送信しない
      }

      //送信データのサイズ
      int ndata=data.size();
      char sendBuf[ndata];//送信バッファ
      //送信バッファにコピー ポインタを使用
      memcpy(sendBuf,&data[0],ndata);

      //送信
      int nWrite=write(clientSock_,sendBuf, sizeof(sendBuf));
      if(nWrite<=0){//Write失敗
        cout<<"[Write] Write Failed:"<<nWrite<<endl;
        perror("Write");
      }

      return nWrite;//送信数を返す
    }

    /**
     *  @brief ソケットを作成する関数
     */
    bool CreateSocket(void){
      //ソケットの設定はサーバもクライアントも一緒
      //引数1：アドレスファミリ AF_INET:IPv4
      //引数2：ソケットタイプ SOCK_STREAM:TCP
      //引数3：プロトコル       
      selfSock_=socket(AF_INET, SOCK_STREAM, 0);
      if(selfSock_<0){//失敗
        cout<<"[CreateSocket] socket failed"<<endl;
        perror("CreateSocket");
        return false;
      }
      else{//成功
        cout<<"[CreateSocket] Create Socket"<<endl;
        return true;
      }
    }

    /**
     *  @brief サーバのデータ構造体を取得する関数
     *         (クライアント,サーバ共通)
     */
    sockaddr_in GetServerInfo(void){
      sockaddr_in server;
      server.sin_family =AF_INET;//TCP
      //ポート番号の指定 htonsはエンディアンの変換
      server.sin_port   =htons(port_);
      return server;
    }

    /**
     * @brief パケットの中身を16進数で表示する関数
     */
    void ShowPacketContents(const vector<char> &pkt) const {
      int nPkt=pkt.size();
      for(int ip=0;ip<nPkt;ip++){
        if(ip%4==0&&ip!=0){printf(",");}//4byte境界で区切る
        printf("%02x",pkt[ip] & 0xff);
      }
      printf("\n");
    }

  private:
    /**
     *  @brief メンバ変数の初期化関数
     */
    void InitMember(void){
      isInit_=false;//初期化されていない
    }
};

/** 
 * @brief TCP通信クライアント用クラス
 * @note  このクライアントを使用する時は、
 *        オブジェクトを生成したあとInit関数を必ず呼んで下さい
 *        クライアントクラスと重複コードを減らすために
 *        TcpMasterクラスを継承しています。
 * 使用例:
 *        TcpClient client;//TCPクライアント用オブジェクトの作成
 *        string LOCALHOST="127.0.0.1";//ローカルホストのIPアドレス
 *        int PORT=50001;//ポート番号
 *        client.Init(LOCALHOST,PORT);
 *
 *        vector<char> data;
 *        int nSend=tcp.Read(data);//データ受信
 *        int nWrite=tcp.Write(data);//データ送信
 */
class TcpClient:public TcpMaster{
  public:
    TcpClient():TcpMaster(){}

    virtual ~TcpClient(){}

    /**
     *  @brief 初期化関数
     *  @param ip 接続先のサーバのIPアドレス
     *  @param ip 接続先のサーバのポート番号
     *  @return Initが成功したか？
     */
    bool Init(string ip, int port){return InitClient(ip,port);}

  private:
    //=====メンバ変数====
    string ip_;     //IPアドレス

    //=====メンバ関数====
    /**
     *  @brief クライアントの初期化関数
     *  @param ip 接続先のサーバのIPアドレス
     *  @param ip 接続先のサーバのポート番号
     *  @return Initが成功したか？
     */
    bool InitClient(string ip, int port){
      //メンバ変数へのコピー
      ip_   =ip;
      port_ =port;

      //ソケットの作成
      bool isCreated=CreateSocket();
      if(isCreated){//ソケットの作成に成功
        //クライアントクラスの場合
        //selfSockとclientsock_は同じになる
        clientSock_=selfSock_;
      }

      //接続先指定用構造体の作成
      sockaddr_in server=GetServerInfo();

      //IPアドレスの指定 
      //inet_addrは文字列のポインタから、バイナリ値に変換している。
      server.sin_addr.s_addr =inet_addr(ip_.c_str());

      //サーバに接続
      isInit_=Connect(server);
      return isInit_;
    }

    /**
     *  @brief サーバに接続する関数
     *  @return サーバに接続できたかどうか
     */
    bool Connect(const sockaddr_in &server){
      int conResult=connect(selfSock_,(struct sockaddr *)&server,sizeof(server));
      if(conResult<0){//失敗
	cout<<"Failed to open TCP Connection to "<<ip_<<endl;
	perror("Connect");
        return false;
      }
      else{//成功
        cout<<"Sucsess to open TCP Connection to "<<ip_<<endl;
        return true;
      }
    }
};

/** 
 * @brief TCP通信サーバ用クラス 
 * @note  このクライアントを使用する時は、
 *        オブジェクトを生成したあとInit関数を必ず呼んで下さい
 *        クライアントクラスと重複コードを減らすために
 *        TcpMasterクラスを継承しています。
 * 使用方法:
 *        TcpServer server;//TCPサーバ用オブジェクト
 *        int PORT=50001;//サーバのポート番号
 *        server.Init(PORT);//初期化
 *        
 *        vector<char> data;
 *        int nSend=server.Send(data);  //受信
 *        int nWrite=server.Write(data);//送信
 */
class TcpServer:public TcpMaster{
  public:
    TcpServer():TcpMaster(){}

    virtual ~TcpServer(){
      close(clientSock_);//ソケットを閉じる サーバだけ
    }
    
    /**
     *  @brief 初期化関数
     *  @param サーバのポート番号
     *  @return 初期化が成功したか？
     */
    bool Init(int port){return InitServer(port);}

  private:
    /**
     * @brief サーバ用初期化関数
     * @param サーバのポート番号
     * @return 初期化が成功したか？
     */
    bool InitServer(int port){
      //メンバ変数へのコピー
      port_ =port;

      //ソケットの作成
      CreateSocket();

      //サーバの設定用構造体の作成
      sockaddr_in server=GetServerInfo();
      //IPアドレスの指定(サーバ用設定)
      server.sin_addr.s_addr =INADDR_ANY;//すべてのIPを接続可能に

      //サーバを切った後にTIME WAIT状態にしなくするおまじない
      int yes=1;
      setsockopt(selfSock_,SOL_SOCKET,SO_REUSEADDR,(const char *)&yes,sizeof(yes));

      //サーバの設定(bind)
      int statBind=bind(selfSock_, (struct sockaddr *)&server,sizeof(server));
      if(statBind!=0){//Bind失敗
        cout<<"[Init] Bind Failed:"<<statBind<<endl;
        perror("Bind");
        return false;
      }

      //サーバを接続可能状態に
      //２つ目の引数は接続待ちのクライアントの最大数 通常5-10らしい
      int statListen=listen(selfSock_, 5);
      if(statListen!=0){//Listen失敗
        cout<<"[Init] Listen Failed:"<<statListen<<endl;
        perror("Listen");
        return false;
      }

      //サーバを接続待ちにする
      isInit_=WaitClientConnection();
      return isInit_;//ステータスを返す
    }

    /**
     * @brief クライアントのコネクションを待つ関数
     */
    bool WaitClientConnection(void){ 
      cout<<"Waiting Client...."<<endl;
      sockaddr_in client;
      unsigned int len=sizeof(client);
      clientSock_=accept(selfSock_, (struct sockaddr *)&client,&len);
      if(clientSock_<0){//Accept失敗
        cout<<"[Init] Accept Failed:"<<clientSock_<<endl;
        perror("Accept");
        return false;
      }
      //成功 Connect先のクライアントの情報を表示する
      printf("[Init] Accepted connection from %s, port=%d\n",
            inet_ntoa(client.sin_addr), ntohs(client.sin_port));
      return true;
    }

};

#endif  //TCP_LIB_HPP
