USE [master]
GO
/****** Object:  Database [MANTIS]    Script Date: 28/08/2016 6:59:25 ******/
CREATE DATABASE [MANTIS]
 CONTAINMENT = NONE
 ON  PRIMARY 
( NAME = N'MANTIS', FILENAME = N'C:\Program Files\Microsoft SQL Server\MSSQL11.MSSQLSERVER\MSSQL\DATA\MANTIS.mdf' , SIZE = 5120KB , MAXSIZE = UNLIMITED, FILEGROWTH = 1024KB )
 LOG ON 
( NAME = N'MANTIS_log', FILENAME = N'C:\Program Files\Microsoft SQL Server\MSSQL11.MSSQLSERVER\MSSQL\DATA\MANTIS_log.ldf' , SIZE = 2048KB , MAXSIZE = 2048GB , FILEGROWTH = 10%)
GO
ALTER DATABASE [MANTIS] SET COMPATIBILITY_LEVEL = 110
GO
IF (1 = FULLTEXTSERVICEPROPERTY('IsFullTextInstalled'))
begin
EXEC [MANTIS].[dbo].[sp_fulltext_database] @action = 'enable'
end
GO
ALTER DATABASE [MANTIS] SET ANSI_NULL_DEFAULT OFF 
GO
ALTER DATABASE [MANTIS] SET ANSI_NULLS OFF 
GO
ALTER DATABASE [MANTIS] SET ANSI_PADDING OFF 
GO
ALTER DATABASE [MANTIS] SET ANSI_WARNINGS OFF 
GO
ALTER DATABASE [MANTIS] SET ARITHABORT OFF 
GO
ALTER DATABASE [MANTIS] SET AUTO_CLOSE OFF 
GO
ALTER DATABASE [MANTIS] SET AUTO_CREATE_STATISTICS ON 
GO
ALTER DATABASE [MANTIS] SET AUTO_SHRINK OFF 
GO
ALTER DATABASE [MANTIS] SET AUTO_UPDATE_STATISTICS ON 
GO
ALTER DATABASE [MANTIS] SET CURSOR_CLOSE_ON_COMMIT OFF 
GO
ALTER DATABASE [MANTIS] SET CURSOR_DEFAULT  GLOBAL 
GO
ALTER DATABASE [MANTIS] SET CONCAT_NULL_YIELDS_NULL OFF 
GO
ALTER DATABASE [MANTIS] SET NUMERIC_ROUNDABORT OFF 
GO
ALTER DATABASE [MANTIS] SET QUOTED_IDENTIFIER OFF 
GO
ALTER DATABASE [MANTIS] SET RECURSIVE_TRIGGERS OFF 
GO
ALTER DATABASE [MANTIS] SET  DISABLE_BROKER 
GO
ALTER DATABASE [MANTIS] SET AUTO_UPDATE_STATISTICS_ASYNC OFF 
GO
ALTER DATABASE [MANTIS] SET DATE_CORRELATION_OPTIMIZATION OFF 
GO
ALTER DATABASE [MANTIS] SET TRUSTWORTHY OFF 
GO
ALTER DATABASE [MANTIS] SET ALLOW_SNAPSHOT_ISOLATION OFF 
GO
ALTER DATABASE [MANTIS] SET PARAMETERIZATION SIMPLE 
GO
ALTER DATABASE [MANTIS] SET READ_COMMITTED_SNAPSHOT OFF 
GO
ALTER DATABASE [MANTIS] SET HONOR_BROKER_PRIORITY OFF 
GO
ALTER DATABASE [MANTIS] SET RECOVERY FULL 
GO
ALTER DATABASE [MANTIS] SET  MULTI_USER 
GO
ALTER DATABASE [MANTIS] SET PAGE_VERIFY CHECKSUM  
GO
ALTER DATABASE [MANTIS] SET DB_CHAINING OFF 
GO
ALTER DATABASE [MANTIS] SET FILESTREAM( NON_TRANSACTED_ACCESS = OFF ) 
GO
ALTER DATABASE [MANTIS] SET TARGET_RECOVERY_TIME = 0 SECONDS 
GO
EXEC sys.sp_db_vardecimal_storage_format N'MANTIS', N'ON'
GO
USE [MANTIS]
GO
/****** Object:  StoredProcedure [dbo].[INSERT_CONN_STATUS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 27 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: INSERT KE TABEL CONN_STATUS
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [INSERT_CONN_STATUS] '192.168.0.3', '1'
-- =============================================

CREATE PROCEDURE [dbo].[INSERT_CONN_STATUS] 
	@IP_ADD NVARCHAR(MAX),
	@STATUS INT
AS
BEGIN
	INSERT INTO CONN_STATUS (
		ADDRESS, 
		STATUS, 
		TIMESTAMP)
	VALUES
		(@IP_ADD,
		@STATUS,
		GETDATE()
		)
END
GO
/****** Object:  StoredProcedure [dbo].[INSERT_DATA_SYS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 27 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: INSERT KE TABEL DATA_SYS
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [INSERT_DATA_SYS] '1|1|1|1|1|1|1|1|1'
-- =============================================

CREATE PROCEDURE [dbo].[INSERT_DATA_SYS] 
	@DATA NVARCHAR(MAX)
AS
BEGIN
	DECLARE
		@TILT NVARCHAR(MAX),
		@LDR NVARCHAR(MAX),
		@WL NVARCHAR(MAX),
		@FLAME NVARCHAR(MAX),
		@USIRR NVARCHAR(MAX),
		@DHT NVARCHAR(MAX),
		@SUHU NVARCHAR(MAX),
		@SMOKE NVARCHAR(MAX),
		@POT NVARCHAR(MAX),
		@CMD NVARCHAR(MAX)='',
		@POS INT

	-- GET TILT
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @TILT = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @TILT = 0;

	-- GET LDR
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @LDR = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @LDR = 0;
	
	-- GET WL
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @WL = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @WL = 0;

	-- GET FLAME
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @FLAME = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @FLAME = 0;

	-- GET USIRR
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @USIRR = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @USIRR = 0;

	-- GET DHT
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @DHT = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @DHT = 0;

	-- GET SUHU
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @SUHU = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @SUHU = 0;

	-- GET SMOKE
	SET @POS = PATINDEX('%|%',@DATA)
	IF(@POS <> 0)
	BEGIN
		SET @SMOKE = SUBSTRING(@DATA,0,@POS)
		SET @DATA = SUBSTRING(@DATA, @POS+1, LEN(@DATA)-@POS)
	END
	ELSE
		SET @SMOKE = 0;

	-- GET POT
	IF(@DATA <> '')
		SET @POT = @DATA
	ELSE
		SET @POT = 0;

	SET @CMD += 'INSERT INTO DATA_SYS (
		[TILT]
		,[TILT_DN]
		,[LDR]
		,[LDR_DN]
		,[WL]
		,[WL_DN]
		,[FLAME]
		,[FLAME_DN]
		,[USIRR]
		,[DHT]
		,[SUHU]
		,[SMOKE]
		,[SMOKE_DN]
		,[POT]
		,[POT_DN]
		,[TIMESTAMP])
	VALUES(
		'+(SELECT TOP 1
		REPLACE(TILT,'X',@TILT)+','+
		@TILT+','+
		REPLACE(LDR,'X',@LDR)+','+
		@LDR+','+
		REPLACE(WL,'X',@WL)+','+
		@WL+','+
		REPLACE(FLAME,'X',@FLAME)+','+
		@FLAME+','+
		@USIRR+','+
		@DHT+','+
		@SUHU+','+
		REPLACE(SMOKE,'X',@SMOKE)+','+
		@SMOKE+','+
		REPLACE(POT,'X',@POT)+','+
		@POT+','+
		'GETDATE()'
		FROM
		SENSOR_CONVERT
		ORDER BY TIMESTAMP DESC)+'
		)'

EXEC SP_EXECUTESQL @CMD
END
GO
/****** Object:  StoredProcedure [dbo].[INSERT_SENSOR_CONVERT]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 27 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: INSERT KE TABEL [INSERT_SENSOR_CONVERT]
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [INSERT_SENSOR_CONVERT] 'X', 'X', 'X', 'X', 'X', 'X'
-- =============================================

CREATE PROCEDURE [dbo].[INSERT_SENSOR_CONVERT] 
	@TILT NVARCHAR(MAX),
	@LDR NVARCHAR(MAX),
	@WL NVARCHAR(MAX),
	@FLAME NVARCHAR(MAX),
	@SMOKE NVARCHAR(MAX),
	@POT NVARCHAR(MAX)
AS
BEGIN
	INSERT INTO SENSOR_CONVERT(
		TILT, 
		LDR, 
		WL,
		FLAME,
		SMOKE,
		POT,
		TIMESTAMP)
	VALUES(
		@TILT,
		@LDR,
		@WL,
		@FLAME,
		@SMOKE,
		@POT,
		GETDATE()
		)
END
GO
/****** Object:  StoredProcedure [dbo].[LOAD_CONN_STATUS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 14 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: LOAD DATA DARI TABEL CONN_STATUS
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [LOAD_CONN_STATUS]
-- =============================================

CREATE PROCEDURE [dbo].[LOAD_CONN_STATUS]
AS
BEGIN
	SELECT
		ADDRESS IP_ADDRESS,
		CASE STATUS
			WHEN '0'
				THEN 'DOWN'
			WHEN '1'
				THEN 'UP'
		END CONNECTION_STATUS,
		CONVERT(NVARCHAR(MAX),TIMESTAMP,106) DATE_SYSTEM,
		CONVERT(NVARCHAR(8),TIMESTAMP,108) TIME_SYSTEM
	FROM
		CONN_STATUS
	ORDER BY TIMESTAMP DESC
END
GO
/****** Object:  StoredProcedure [dbo].[LOAD_DATA_SYS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 27 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: LOAD DATA DARI TABEL DATA_SYS
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [LOAD_DATA_SYS]
-- =============================================

CREATE PROCEDURE [dbo].[LOAD_DATA_SYS]
AS
SELECT
	CASE WHEN (STATUS = '1') THEN 'connect' ELSE 'unconnect' END STATUS
	,[TILT]
	,[TILT_DN]
	,[LDR]
	,[LDR_DN]
	,[WL]
	,[WL_DN]
	,[FLAME]
	,[FLAME_DN]
	,[USIRR]
	,[DHT]
	,[SUHU]
	,[SMOKE]
	,[SMOKE_DN]
	,[POT]
	,[POT_DN]
	,AVG_TILT
	,AVG_LDR
	,AVG_WL
	,AVG_FLAME
	,AVG_USIRR
	,AVG_DHT
	,AVG_SUHU
	,AVG_SMOKE
	,AVG_POT
	,MAX_TILT
	,MAX_LDR
	,MAX_WL
	,MAX_FLAME
	,MAX_USIRR
	,MAX_DHT
	,MAX_SUHU
	,MAX_SMOKE
	,MAX_POT
	,MIN_TILT
	,MIN_LDR
	,MIN_WL
	,MIN_FLAME
	,MIN_USIRR
	,MIN_DHT
	,MIN_SUHU
	,MIN_SMOKE
	,MIN_POT
	,STDEV_TILT
	,STDEV_LDR
	,STDEV_WL
	,STDEV_FLAME
	,STDEV_USIRR
	,STDEV_DHT
	,STDEV_SUHU
	,STDEV_SMOKE
	,STDEV_POT
FROM
	(SELECT TOP 1
		[TILT]
		,[TILT_DN]
		,[LDR]
		,[LDR_DN]
		,[WL]
		,[WL_DN]
		,[FLAME]
		,[FLAME_DN]
		,[USIRR]
		,[DHT]
		,[SUHU]
		,[SMOKE]
		,[SMOKE_DN]
		,[POT]
		,[POT_DN]
		, 1 ID
	FROM
		DATA_SYS
	ORDER BY
		TIMESTAMP DESC) A
	LEFT JOIN
	(SELECT 
		CAST(AVG([TILT]) AS DEC(6,2)) AVG_TILT
		,CAST(AVG([LDR]) AS DEC(6,2)) AVG_LDR
		,CAST(AVG([WL]) AS DEC(6,2)) AVG_WL
		,CAST(AVG([FLAME]) AS DEC(6,2)) AVG_FLAME
		,CAST(AVG([USIRR]) AS DEC(6,2)) AVG_USIRR
		,CAST(AVG([DHT]) AS DEC(6,2)) AVG_DHT
		,CAST(AVG([SUHU]) AS DEC(6,2)) AVG_SUHU
		,CAST(AVG([SMOKE]) AS DEC(6,2)) AVG_SMOKE
		,CAST(AVG([POT]) AS DEC(6,2)) AVG_POT
	
		,CAST(MAX([TILT]) AS DEC(6,2)) MAX_TILT
		,CAST(MAX([LDR]) AS DEC(6,2)) MAX_LDR
		,CAST(MAX([WL]) AS DEC(6,2)) MAX_WL
		,CAST(MAX([FLAME]) AS DEC(6,2)) MAX_FLAME
		,CAST(MAX([USIRR]) AS DEC(6,2)) MAX_USIRR
		,CAST(MAX([DHT]) AS DEC(6,2)) MAX_DHT
		,CAST(MAX([SUHU]) AS DEC(6,2)) MAX_SUHU
		,CAST(MAX([SMOKE]) AS DEC(6,2)) MAX_SMOKE
		,CAST(MAX([POT]) AS DEC(6,2)) MAX_POT
	
		,CAST(MIN([TILT]) AS DEC(6,2)) MIN_TILT
		,CAST(MIN([LDR]) AS DEC(6,2)) MIN_LDR
		,CAST(MIN([WL]) AS DEC(6,2)) MIN_WL
		,CAST(MIN([FLAME]) AS DEC(6,2)) MIN_FLAME
		,CAST(MIN([USIRR]) AS DEC(6,2)) MIN_USIRR
		,CAST(MIN([DHT]) AS DEC(6,2)) MIN_DHT
		,CAST(MIN([SUHU]) AS DEC(6,2)) MIN_SUHU
		,CAST(MIN([SMOKE]) AS DEC(6,2)) MIN_SMOKE
		,CAST(MIN([POT]) AS DEC(6,2)) MIN_POT
	
		,CAST(STDEV([TILT]) AS DEC(6,2)) STDEV_TILT
		,CAST(STDEV([LDR]) AS DEC(6,2)) STDEV_LDR
		,CAST(STDEV([WL]) AS DEC(6,2)) STDEV_WL
		,CAST(STDEV([FLAME]) AS DEC(6,2)) STDEV_FLAME
		,CAST(STDEV([USIRR]) AS DEC(6,2)) STDEV_USIRR
		,CAST(STDEV([DHT]) AS DEC(6,2)) STDEV_DHT
		,CAST(STDEV([SUHU]) AS DEC(6,2)) STDEV_SUHU
		,CAST(STDEV([SMOKE]) AS DEC(6,2)) STDEV_SMOKE
		,CAST(STDEV([POT]) AS DEC(6,2)) STDEV_POT
		, 1 ID
	FROM
		DATA_SYS) B ON A.ID = B.ID
LEFT JOIN
	(SELECT TOP 1
		STATUS
		, 1 ID
	FROM
		CONN_STATUS
	ORDER BY
		TIMESTAMP DESC) C ON A.ID = C.ID
GO
/****** Object:  StoredProcedure [dbo].[LOAD_LIST_USIRR]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- EXEC [LOAD_LIST_USIRR]
CREATE PROCEDURE [dbo].[LOAD_LIST_USIRR]
AS
SELECT TOP 100
	CONVERT(NVARCHAR(10),TIMESTAMP,103),--ROW_NUMBER() OVER (ORDER BY TIMESTAMP DESC) AS NO,
	USIRR
FROM
	DATA_SYS
GO
/****** Object:  StoredProcedure [dbo].[LOAD_SENSOR_CONVERT]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO

-- =============================================
-- AUTHOR		: NOER FITRIA PUTRA SETYONO
-- CREATE DATE	: 27 AGUSTUS 2016
-- UPDATE DATE	: 
-- UPDATE BY	:
-- DESCRIPTION	: LOAD DATA DARI TABEL SENSOR_CONVERT
-- UPDATE DESC	:
-- HOW TO USE	: EXEC [LOAD_SENSOR_CONVERT]
-- =============================================

CREATE PROCEDURE [dbo].[LOAD_SENSOR_CONVERT]
AS
BEGIN
	SELECT TOP 1
		TILT,
		LDR,
		WL,
		FLAME,
		SMOKE,
		POT
	FROM
		SENSOR_CONVERT
	ORDER BY
		TIMESTAMP DESC
END
GO
/****** Object:  Table [dbo].[CONN_STATUS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[CONN_STATUS](
	[ADDRESS] [nvarchar](max) NULL,
	[STATUS] [bit] NULL,
	[TIMESTAMP] [datetime] NULL
) ON [PRIMARY] TEXTIMAGE_ON [PRIMARY]

GO
/****** Object:  Table [dbo].[DATA_SYS]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[DATA_SYS](
	[TILT] [decimal](4, 0) NULL,
	[TILT_DN] [decimal](4, 0) NULL,
	[LDR] [decimal](4, 0) NULL,
	[LDR_DN] [decimal](4, 0) NULL,
	[WL] [decimal](4, 0) NULL,
	[WL_DN] [decimal](4, 0) NULL,
	[FLAME] [decimal](4, 0) NULL,
	[FLAME_DN] [decimal](4, 0) NULL,
	[USIRR] [decimal](4, 0) NULL,
	[DHT] [decimal](4, 0) NULL,
	[SUHU] [decimal](4, 0) NULL,
	[SMOKE] [decimal](4, 0) NULL,
	[SMOKE_DN] [decimal](4, 0) NULL,
	[POT] [decimal](4, 0) NULL,
	[POT_DN] [decimal](4, 0) NULL,
	[TIMESTAMP] [datetime] NULL
) ON [PRIMARY]

GO
/****** Object:  Table [dbo].[SENSOR_CONVERT]    Script Date: 28/08/2016 6:59:25 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[SENSOR_CONVERT](
	[TILT] [nvarchar](max) NULL,
	[LDR] [nvarchar](max) NULL,
	[WL] [nvarchar](max) NULL,
	[FLAME] [nvarchar](max) NULL,
	[SMOKE] [nvarchar](max) NULL,
	[POT] [nvarchar](max) NULL,
	[TIMESTAMP] [datetime] NULL
) ON [PRIMARY] TEXTIMAGE_ON [PRIMARY]

GO
USE [master]
GO
ALTER DATABASE [MANTIS] SET  READ_WRITE 
GO
