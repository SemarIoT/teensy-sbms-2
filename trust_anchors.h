#ifndef _CERTIFICATES_H_
#define _CERTIFICATES_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* This file is auto-generated by the pycert_bearssl tool.  Do not change it manually.
 * Certificates are BearSSL br_x509_trust_anchor format.  Included certs:
 *
 * Index:    0
 * Label:    AAA Certificate Services
 * Subject:  CN=AAA Certificate Services,O=Comodo CA Limited,L=Salford,ST=Greater Manchester,C=GB
 * Domain(s): iotlab-uns.com
 * 
 * Index:    1
 * Label:    GlobalSign Root CA
 * Subject:  CN=GlobalSign Root CA,OU=Root CA,O=GlobalSign nv-sa,C=BE
 * Domain(s): www.google.com
 * 
 * Index:    2
 * Label:    VeriSign Class 3 Public Primary Certification Authority - G5
 * Subject:  CN=VeriSign Class 3 Public Primary Certification Authority - G5,OU=(c) 2006 VeriSign\, Inc. - For authorized use only,OU=VeriSign Trust Network,O=VeriSign\, Inc.,C=US
 * Domain(s): www.amazon.com
 */

#define TAs_NUM 3

static const unsigned char TA_DN0[] = {
    0x30, 0x7b, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13,
    0x02, 0x47, 0x42, 0x31, 0x1b, 0x30, 0x19, 0x06, 0x03, 0x55, 0x04, 0x08,
    0x0c, 0x12, 0x47, 0x72, 0x65, 0x61, 0x74, 0x65, 0x72, 0x20, 0x4d, 0x61,
    0x6e, 0x63, 0x68, 0x65, 0x73, 0x74, 0x65, 0x72, 0x31, 0x10, 0x30, 0x0e,
    0x06, 0x03, 0x55, 0x04, 0x07, 0x0c, 0x07, 0x53, 0x61, 0x6c, 0x66, 0x6f,
    0x72, 0x64, 0x31, 0x1a, 0x30, 0x18, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c,
    0x11, 0x43, 0x6f, 0x6d, 0x6f, 0x64, 0x6f, 0x20, 0x43, 0x41, 0x20, 0x4c,
    0x69, 0x6d, 0x69, 0x74, 0x65, 0x64, 0x31, 0x21, 0x30, 0x1f, 0x06, 0x03,
    0x55, 0x04, 0x03, 0x0c, 0x18, 0x41, 0x41, 0x41, 0x20, 0x43, 0x65, 0x72,
    0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74, 0x65, 0x20, 0x53, 0x65, 0x72,
    0x76, 0x69, 0x63, 0x65, 0x73,
};

static const unsigned char TA_RSA_N0[] = {
    0xbe, 0x40, 0x9d, 0xf4, 0x6e, 0xe1, 0xea, 0x76, 0x87, 0x1c, 0x4d, 0x45,
    0x44, 0x8e, 0xbe, 0x46, 0xc8, 0x83, 0x06, 0x9d, 0xc1, 0x2a, 0xfe, 0x18,
    0x1f, 0x8e, 0xe4, 0x02, 0xfa, 0xf3, 0xab, 0x5d, 0x50, 0x8a, 0x16, 0x31,
    0x0b, 0x9a, 0x06, 0xd0, 0xc5, 0x70, 0x22, 0xcd, 0x49, 0x2d, 0x54, 0x63,
    0xcc, 0xb6, 0x6e, 0x68, 0x46, 0x0b, 0x53, 0xea, 0xcb, 0x4c, 0x24, 0xc0,
    0xbc, 0x72, 0x4e, 0xea, 0xf1, 0x15, 0xae, 0xf4, 0x54, 0x9a, 0x12, 0x0a,
    0xc3, 0x7a, 0xb2, 0x33, 0x60, 0xe2, 0xda, 0x89, 0x55, 0xf3, 0x22, 0x58,
    0xf3, 0xde, 0xdc, 0xcf, 0xef, 0x83, 0x86, 0xa2, 0x8c, 0x94, 0x4f, 0x9f,
    0x68, 0xf2, 0x98, 0x90, 0x46, 0x84, 0x27, 0xc7, 0x76, 0xbf, 0xe3, 0xcc,
    0x35, 0x2c, 0x8b, 0x5e, 0x07, 0x64, 0x65, 0x82, 0xc0, 0x48, 0xb0, 0xa8,
    0x91, 0xf9, 0x61, 0x9f, 0x76, 0x20, 0x50, 0xa8, 0x91, 0xc7, 0x66, 0xb5,
    0xeb, 0x78, 0x62, 0x03, 0x56, 0xf0, 0x8a, 0x1a, 0x13, 0xea, 0x31, 0xa3,
    0x1e, 0xa0, 0x99, 0xfd, 0x38, 0xf6, 0xf6, 0x27, 0x32, 0x58, 0x6f, 0x07,
    0xf5, 0x6b, 0xb8, 0xfb, 0x14, 0x2b, 0xaf, 0xb7, 0xaa, 0xcc, 0xd6, 0x63,
    0x5f, 0x73, 0x8c, 0xda, 0x05, 0x99, 0xa8, 0x38, 0xa8, 0xcb, 0x17, 0x78,
    0x36, 0x51, 0xac, 0xe9, 0x9e, 0xf4, 0x78, 0x3a, 0x8d, 0xcf, 0x0f, 0xd9,
    0x42, 0xe2, 0x98, 0x0c, 0xab, 0x2f, 0x9f, 0x0e, 0x01, 0xde, 0xef, 0x9f,
    0x99, 0x49, 0xf1, 0x2d, 0xdf, 0xac, 0x74, 0x4d, 0x1b, 0x98, 0xb5, 0x47,
    0xc5, 0xe5, 0x29, 0xd1, 0xf9, 0x90, 0x18, 0xc7, 0x62, 0x9c, 0xbe, 0x83,
    0xc7, 0x26, 0x7b, 0x3e, 0x8a, 0x25, 0xc7, 0xc0, 0xdd, 0x9d, 0xe6, 0x35,
    0x68, 0x10, 0x20, 0x9d, 0x8f, 0xd8, 0xde, 0xd2, 0xc3, 0x84, 0x9c, 0x0d,
    0x5e, 0xe8, 0x2f, 0xc9,
};

static const unsigned char TA_RSA_E0[] = {
    0x01, 0x00, 0x01,
};

static const unsigned char TA_DN1[] = {
    0x30, 0x57, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13,
    0x02, 0x42, 0x45, 0x31, 0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x0a,
    0x13, 0x10, 0x47, 0x6c, 0x6f, 0x62, 0x61, 0x6c, 0x53, 0x69, 0x67, 0x6e,
    0x20, 0x6e, 0x76, 0x2d, 0x73, 0x61, 0x31, 0x10, 0x30, 0x0e, 0x06, 0x03,
    0x55, 0x04, 0x0b, 0x13, 0x07, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41,
    0x31, 0x1b, 0x30, 0x19, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x12, 0x47,
    0x6c, 0x6f, 0x62, 0x61, 0x6c, 0x53, 0x69, 0x67, 0x6e, 0x20, 0x52, 0x6f,
    0x6f, 0x74, 0x20, 0x43, 0x41,
};

static const unsigned char TA_RSA_N1[] = {
    0xda, 0x0e, 0xe6, 0x99, 0x8d, 0xce, 0xa3, 0xe3, 0x4f, 0x8a, 0x7e, 0xfb,
    0xf1, 0x8b, 0x83, 0x25, 0x6b, 0xea, 0x48, 0x1f, 0xf1, 0x2a, 0xb0, 0xb9,
    0x95, 0x11, 0x04, 0xbd, 0xf0, 0x63, 0xd1, 0xe2, 0x67, 0x66, 0xcf, 0x1c,
    0xdd, 0xcf, 0x1b, 0x48, 0x2b, 0xee, 0x8d, 0x89, 0x8e, 0x9a, 0xaf, 0x29,
    0x80, 0x65, 0xab, 0xe9, 0xc7, 0x2d, 0x12, 0xcb, 0xab, 0x1c, 0x4c, 0x70,
    0x07, 0xa1, 0x3d, 0x0a, 0x30, 0xcd, 0x15, 0x8d, 0x4f, 0xf8, 0xdd, 0xd4,
    0x8c, 0x50, 0x15, 0x1c, 0xef, 0x50, 0xee, 0xc4, 0x2e, 0xf7, 0xfc, 0xe9,
    0x52, 0xf2, 0x91, 0x7d, 0xe0, 0x6d, 0xd5, 0x35, 0x30, 0x8e, 0x5e, 0x43,
    0x73, 0xf2, 0x41, 0xe9, 0xd5, 0x6a, 0xe3, 0xb2, 0x89, 0x3a, 0x56, 0x39,
    0x38, 0x6f, 0x06, 0x3c, 0x88, 0x69, 0x5b, 0x2a, 0x4d, 0xc5, 0xa7, 0x54,
    0xb8, 0x6c, 0x89, 0xcc, 0x9b, 0xf9, 0x3c, 0xca, 0xe5, 0xfd, 0x89, 0xf5,
    0x12, 0x3c, 0x92, 0x78, 0x96, 0xd6, 0xdc, 0x74, 0x6e, 0x93, 0x44, 0x61,
    0xd1, 0x8d, 0xc7, 0x46, 0xb2, 0x75, 0x0e, 0x86, 0xe8, 0x19, 0x8a, 0xd5,
    0x6d, 0x6c, 0xd5, 0x78, 0x16, 0x95, 0xa2, 0xe9, 0xc8, 0x0a, 0x38, 0xeb,
    0xf2, 0x24, 0x13, 0x4f, 0x73, 0x54, 0x93, 0x13, 0x85, 0x3a, 0x1b, 0xbc,
    0x1e, 0x34, 0xb5, 0x8b, 0x05, 0x8c, 0xb9, 0x77, 0x8b, 0xb1, 0xdb, 0x1f,
    0x20, 0x91, 0xab, 0x09, 0x53, 0x6e, 0x90, 0xce, 0x7b, 0x37, 0x74, 0xb9,
    0x70, 0x47, 0x91, 0x22, 0x51, 0x63, 0x16, 0x79, 0xae, 0xb1, 0xae, 0x41,
    0x26, 0x08, 0xc8, 0x19, 0x2b, 0xd1, 0x46, 0xaa, 0x48, 0xd6, 0x64, 0x2a,
    0xd7, 0x83, 0x34, 0xff, 0x2c, 0x2a, 0xc1, 0x6c, 0x19, 0x43, 0x4a, 0x07,
    0x85, 0xe7, 0xd3, 0x7c, 0xf6, 0x21, 0x68, 0xef, 0xea, 0xf2, 0x52, 0x9f,
    0x7f, 0x93, 0x90, 0xcf,
};

static const unsigned char TA_RSA_E1[] = {
    0x01, 0x00, 0x01,
};

static const unsigned char TA_DN2[] = {
    0x30, 0x81, 0xca, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06,
    0x13, 0x02, 0x55, 0x53, 0x31, 0x17, 0x30, 0x15, 0x06, 0x03, 0x55, 0x04,
    0x0a, 0x13, 0x0e, 0x56, 0x65, 0x72, 0x69, 0x53, 0x69, 0x67, 0x6e, 0x2c,
    0x20, 0x49, 0x6e, 0x63, 0x2e, 0x31, 0x1f, 0x30, 0x1d, 0x06, 0x03, 0x55,
    0x04, 0x0b, 0x13, 0x16, 0x56, 0x65, 0x72, 0x69, 0x53, 0x69, 0x67, 0x6e,
    0x20, 0x54, 0x72, 0x75, 0x73, 0x74, 0x20, 0x4e, 0x65, 0x74, 0x77, 0x6f,
    0x72, 0x6b, 0x31, 0x3a, 0x30, 0x38, 0x06, 0x03, 0x55, 0x04, 0x0b, 0x13,
    0x31, 0x28, 0x63, 0x29, 0x20, 0x32, 0x30, 0x30, 0x36, 0x20, 0x56, 0x65,
    0x72, 0x69, 0x53, 0x69, 0x67, 0x6e, 0x2c, 0x20, 0x49, 0x6e, 0x63, 0x2e,
    0x20, 0x2d, 0x20, 0x46, 0x6f, 0x72, 0x20, 0x61, 0x75, 0x74, 0x68, 0x6f,
    0x72, 0x69, 0x7a, 0x65, 0x64, 0x20, 0x75, 0x73, 0x65, 0x20, 0x6f, 0x6e,
    0x6c, 0x79, 0x31, 0x45, 0x30, 0x43, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13,
    0x3c, 0x56, 0x65, 0x72, 0x69, 0x53, 0x69, 0x67, 0x6e, 0x20, 0x43, 0x6c,
    0x61, 0x73, 0x73, 0x20, 0x33, 0x20, 0x50, 0x75, 0x62, 0x6c, 0x69, 0x63,
    0x20, 0x50, 0x72, 0x69, 0x6d, 0x61, 0x72, 0x79, 0x20, 0x43, 0x65, 0x72,
    0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x20, 0x41,
    0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 0x74, 0x79, 0x20, 0x2d, 0x20, 0x47,
    0x35,
};

static const unsigned char TA_RSA_N2[] = {
    0xaf, 0x24, 0x08, 0x08, 0x29, 0x7a, 0x35, 0x9e, 0x60, 0x0c, 0xaa, 0xe7,
    0x4b, 0x3b, 0x4e, 0xdc, 0x7c, 0xbc, 0x3c, 0x45, 0x1c, 0xbb, 0x2b, 0xe0,
    0xfe, 0x29, 0x02, 0xf9, 0x57, 0x08, 0xa3, 0x64, 0x85, 0x15, 0x27, 0xf5,
    0xf1, 0xad, 0xc8, 0x31, 0x89, 0x5d, 0x22, 0xe8, 0x2a, 0xaa, 0xa6, 0x42,
    0xb3, 0x8f, 0xf8, 0xb9, 0x55, 0xb7, 0xb1, 0xb7, 0x4b, 0xb3, 0xfe, 0x8f,
    0x7e, 0x07, 0x57, 0xec, 0xef, 0x43, 0xdb, 0x66, 0x62, 0x15, 0x61, 0xcf,
    0x60, 0x0d, 0xa4, 0xd8, 0xde, 0xf8, 0xe0, 0xc3, 0x62, 0x08, 0x3d, 0x54,
    0x13, 0xeb, 0x49, 0xca, 0x59, 0x54, 0x85, 0x26, 0xe5, 0x2b, 0x8f, 0x1b,
    0x9f, 0xeb, 0xf5, 0xa1, 0x91, 0xc2, 0x33, 0x49, 0xd8, 0x43, 0x63, 0x6a,
    0x52, 0x4b, 0xd2, 0x8f, 0xe8, 0x70, 0x51, 0x4d, 0xd1, 0x89, 0x69, 0x7b,
    0xc7, 0x70, 0xf6, 0xb3, 0xdc, 0x12, 0x74, 0xdb, 0x7b, 0x5d, 0x4b, 0x56,
    0xd3, 0x96, 0xbf, 0x15, 0x77, 0xa1, 0xb0, 0xf4, 0xa2, 0x25, 0xf2, 0xaf,
    0x1c, 0x92, 0x67, 0x18, 0xe5, 0xf4, 0x06, 0x04, 0xef, 0x90, 0xb9, 0xe4,
    0x00, 0xe4, 0xdd, 0x3a, 0xb5, 0x19, 0xff, 0x02, 0xba, 0xf4, 0x3c, 0xee,
    0xe0, 0x8b, 0xeb, 0x37, 0x8b, 0xec, 0xf4, 0xd7, 0xac, 0xf2, 0xf6, 0xf0,
    0x3d, 0xaf, 0xdd, 0x75, 0x91, 0x33, 0x19, 0x1d, 0x1c, 0x40, 0xcb, 0x74,
    0x24, 0x19, 0x21, 0x93, 0xd9, 0x14, 0xfe, 0xac, 0x2a, 0x52, 0xc7, 0x8f,
    0xd5, 0x04, 0x49, 0xe4, 0x8d, 0x63, 0x47, 0x88, 0x3c, 0x69, 0x83, 0xcb,
    0xfe, 0x47, 0xbd, 0x2b, 0x7e, 0x4f, 0xc5, 0x95, 0xae, 0x0e, 0x9d, 0xd4,
    0xd1, 0x43, 0xc0, 0x67, 0x73, 0xe3, 0x14, 0x08, 0x7e, 0xe5, 0x3f, 0x9f,
    0x73, 0xb8, 0x33, 0x0a, 0xcf, 0x5d, 0x3f, 0x34, 0x87, 0x96, 0x8a, 0xee,
    0x53, 0xe8, 0x25, 0x15,
};

static const unsigned char TA_RSA_E2[] = {
    0x01, 0x00, 0x01,
};

static const br_x509_trust_anchor TAs[] = {
    {
        { (unsigned char *)TA_DN0, sizeof TA_DN0 },
        BR_X509_TA_CA,
        {
            BR_KEYTYPE_RSA,
            { .rsa = {
                (unsigned char *)TA_RSA_N0, sizeof TA_RSA_N0,
                (unsigned char *)TA_RSA_E0, sizeof TA_RSA_E0,
            } }
        }
    },
    {
        { (unsigned char *)TA_DN1, sizeof TA_DN1 },
        BR_X509_TA_CA,
        {
            BR_KEYTYPE_RSA,
            { .rsa = {
                (unsigned char *)TA_RSA_N1, sizeof TA_RSA_N1,
                (unsigned char *)TA_RSA_E1, sizeof TA_RSA_E1,
            } }
        }
    },
    {
        { (unsigned char *)TA_DN2, sizeof TA_DN2 },
        BR_X509_TA_CA,
        {
            BR_KEYTYPE_RSA,
            { .rsa = {
                (unsigned char *)TA_RSA_N2, sizeof TA_RSA_N2,
                (unsigned char *)TA_RSA_E2, sizeof TA_RSA_E2,
            } }
        }
    },
};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef _CERTIFICATES_H_ */
