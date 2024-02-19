typedef enum {
    EE_STATUS_FLASH_FAIL = 0, // indicates failure in hal functions
    EE_STATUS_PAGE_UNDEF,
    EE_STATUS_PAGE_EMPTY,
    EE_STATUS_PAGE_FULL,
    EE_STATUS_OK
} EE_STATUS_ENUM;

EE_STATUS_ENUM ee_init(void)
{
    return EE_STATUS_OK;
}

EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen)
{
    return EE_STATUS_OK;
}

EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen)
{
    return EE_STATUS_OK;
}
