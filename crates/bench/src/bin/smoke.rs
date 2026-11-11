//! 교차 빌드 확인 (미리 재기, docs/status.md).
//!
//! PC(Windows)에서 파이용으로 교차 빌드한 실행 파일이 파이에서 도는지 본다. 컨트롤러가 쓸 것 가운데
//! 교차 빌드에서 막히기 쉬운 것을 한 번씩 써 본다: C 소스를 같이 빌드하는 TLS 암호 구현,
//! 시리얼 포트 목록, 게임패드 같은 입력 장치 목록.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::process::ExitCode;
use std::sync::Arc;
use std::thread;

use rustls::crypto::CryptoProvider;
use rustls::pki_types::{PrivateKeyDer, PrivatePkcs8KeyDer, ServerName};
use rustls::{
    ClientConfig, ClientConnection, RootCertStore, ServerConfig, ServerConnection, StreamOwned,
};

type Error = Box<dyn std::error::Error + Send + Sync>;
type Check = fn() -> Result<String, Error>;

#[cfg(not(any(feature = "ring", feature = "aws-lc-rs")))]
compile_error!("TLS 암호 구현 기능(ring 또는 aws-lc-rs)을 하나 켠다");

#[cfg(feature = "ring")]
const PROVIDER: &str = "ring";
#[cfg(feature = "ring")]
fn provider() -> CryptoProvider {
    rustls::crypto::ring::default_provider()
}

#[cfg(all(feature = "aws-lc-rs", not(feature = "ring")))]
const PROVIDER: &str = "aws-lc-rs";
#[cfg(all(feature = "aws-lc-rs", not(feature = "ring")))]
fn provider() -> CryptoProvider {
    rustls::crypto::aws_lc_rs::default_provider()
}

fn main() -> ExitCode {
    let linking = if cfg!(target_env = "musl") {
        "musl 정적 실행 파일"
    } else {
        "동적 실행 파일"
    };
    println!(
        "대상: {}-{} ({linking})",
        std::env::consts::ARCH,
        std::env::consts::OS
    );
    let checks: [(&str, Check); 4] = [
        ("시스템", system),
        ("TLS", tls),
        ("시리얼 포트", serial),
        ("입력 장치", input),
    ];
    let mut failed = 0;
    for (name, check) in checks {
        match check() {
            Ok(text) => println!("[통과] {name}: {text}"),
            Err(error) => {
                failed += 1;
                println!("[실패] {name}: {error}");
            }
        }
    }
    if failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn system() -> Result<String, Error> {
    let cores = thread::available_parallelism()?.get();
    Ok(format!("코어 {cores}개, 커널 {}", kernel()?))
}

#[cfg(target_os = "linux")]
fn kernel() -> Result<String, Error> {
    // 실시간 커널이면 PREEMPT_RT가 보인다.
    Ok(std::fs::read_to_string("/proc/version")?.trim().to_owned())
}

#[cfg(not(target_os = "linux"))]
fn kernel() -> Result<String, Error> {
    Ok("리눅스가 아니라 보지 않는다".to_owned())
}

/// 자체 서명 인증서로 같은 프로세스 안의 TLS 서버와 클라이언트가 한 줄을 주고받는다.
fn tls() -> Result<String, Error> {
    let provider = Arc::new(provider());
    let certified = rcgen::generate_simple_self_signed(vec!["localhost".to_owned()])?;
    let cert = certified.cert.der().clone();
    let key = PrivateKeyDer::Pkcs8(PrivatePkcs8KeyDer::from(
        certified.signing_key.serialize_der(),
    ));

    let server_config = Arc::new(
        ServerConfig::builder_with_provider(Arc::clone(&provider))
            .with_safe_default_protocol_versions()?
            .with_no_client_auth()
            .with_single_cert(vec![cert.clone()], key)?,
    );
    let mut roots = RootCertStore::empty();
    roots.add(cert)?;
    let client_config = Arc::new(
        ClientConfig::builder_with_provider(provider)
            .with_safe_default_protocol_versions()?
            .with_root_certificates(roots)
            .with_no_client_auth(),
    );

    let listener = TcpListener::bind("127.0.0.1:0")?;
    let address = listener.local_addr()?;
    let server = thread::spawn(move || -> Result<(), Error> {
        let (socket, _) = listener.accept()?;
        let mut stream = StreamOwned::new(ServerConnection::new(server_config)?, socket);
        let mut line = String::new();
        BufReader::new(&mut stream).read_line(&mut line)?;
        stream.write_all(line.as_bytes())?;
        stream.flush()?;
        Ok(())
    });

    let socket = TcpStream::connect(address)?;
    let name = ServerName::try_from("localhost")?;
    let mut stream = StreamOwned::new(ClientConnection::new(client_config, name)?, socket);
    stream.write_all(b"ping\n")?;
    stream.flush()?;
    let mut reply = String::new();
    BufReader::new(&mut stream).read_line(&mut reply)?;
    server.join().map_err(|_| "TLS 서버 스레드가 멈췄다")??;
    if reply != "ping\n" {
        return Err(format!("받은 줄이 다르다: {reply:?}").into());
    }
    let version = stream.conn.protocol_version().ok_or("TLS 판을 모른다")?;
    let suite = stream
        .conn
        .negotiated_cipher_suite()
        .ok_or("암호 묶음을 모른다")?;
    Ok(format!(
        "{PROVIDER} 암호 구현으로 왕복했다 ({version:?}, {:?})",
        suite.suite()
    ))
}

fn serial() -> Result<String, Error> {
    let names = horilab_servo::port_names()?;
    Ok(if names.is_empty() {
        "없음 (어댑터를 꽂으면 보인다)".to_owned()
    } else {
        names.join(", ")
    })
}

#[cfg(target_os = "linux")]
fn input() -> Result<String, Error> {
    let names: Vec<String> = evdev::enumerate()
        .map(|(path, device)| {
            let name = device.name().unwrap_or("이름 없음");
            format!("{name} ({})", path.display())
        })
        .collect();
    Ok(if names.is_empty() {
        "없음 (장치가 없거나 읽을 권한이 없다: input 그룹)".to_owned()
    } else {
        names.join(", ")
    })
}

#[cfg(not(target_os = "linux"))]
fn input() -> Result<String, Error> {
    Ok("리눅스가 아니라 보지 않는다".to_owned())
}
